// communication_node.cpp
// the only node that touches the UDP socket
//
// listens on  /cmd_drive     (Twist: linear.x = motor pwm, angular.z = steering deg)
// publishes   /car_feedback  (Int32MultiArray: left ticks, right ticks, fwd cm/s, side cm/s)
//
// everything that wants to drive the car publishes /cmd_drive instead of
// opening its own socket. two nodes writing to the same port interleave their
// packets and the car twitches, which looks like a control bug but isnt

#include "communication/udp_protocol.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <atomic>
#include <cmath>
#include <cstring>
#include <string>
#include <thread>

using communication::DriveCommand;
using communication::Feedback;

class CommunicationNode : public rclcpp::Node {
public:
    CommunicationNode() : rclcpp::Node("communication_node") {
        car_ip_         = declare_parameter<std::string>("remote_host", "127.0.0.1");
        port_to_car_    = declare_parameter<int>("send_port", 12151);
        port_from_car_  = declare_parameter<int>("recv_port", 12152);
        send_rate_hz_   = declare_parameter<double>("rate_hz", 20.0);
        command_timeout_sec_ = declare_parameter<double>("command_timeout_s", 0.5);

        if (!open_sockets()) {
            throw std::runtime_error("could not open the UDP sockets");
        }

        command_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_drive", 10,
            [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
                latest_command_.motor_pwm    = (int)lround(msg->linear.x);
                latest_command_.steering_deg = (int)lround(msg->angular.z);
                last_command_time_ = now();
                got_a_command_ = true;
            });

        feedback_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>("/car_feedback", 10);

        auto period = std::chrono::duration<double>(1.0 / send_rate_hz_);
        send_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            [this] { send_command_to_car(); });

        receiver_running_ = true;
        receiver_thread_ = std::thread([this] { receive_from_car_loop(); });

        RCLCPP_INFO(get_logger(), "sending to %s:%d, listening on :%d, %.1f Hz",
                    car_ip_.c_str(), port_to_car_, port_from_car_, send_rate_hz_);
    }

    ~CommunicationNode() override {
        // order matters here. clear the flag, close the socket so a blocked
        // recvfrom wakes up, then join. joining first would hang
        receiver_running_ = false;
        if (socket_from_car_ >= 0) {
            shutdown(socket_from_car_, SHUT_RDWR);
            close(socket_from_car_);
            socket_from_car_ = -1;
        }
        if (receiver_thread_.joinable()) receiver_thread_.join();
        if (socket_to_car_ >= 0) {
            close(socket_to_car_);
            socket_to_car_ = -1;
        }
    }

private:
    bool open_sockets() {
        socket_to_car_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_to_car_ < 0) {
            RCLCPP_ERROR(get_logger(), "send socket failed: %s", strerror(errno));
            return false;
        }

        memset(&car_address_, 0, sizeof(car_address_));
        car_address_.sin_family = AF_INET;
        car_address_.sin_port = htons((uint16_t)port_to_car_);
        if (inet_pton(AF_INET, car_ip_.c_str(), &car_address_.sin_addr) != 1) {
            RCLCPP_ERROR(get_logger(), "bad remote_host: %s", car_ip_.c_str());
            return false;
        }

        socket_from_car_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_from_car_ < 0) {
            RCLCPP_ERROR(get_logger(), "receive socket failed: %s", strerror(errno));
            return false;
        }

        int reuse = 1;
        setsockopt(socket_from_car_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

        // give the read a timeout so the thread cant block forever on shutdown
        timeval read_timeout;
        read_timeout.tv_sec = 0;
        read_timeout.tv_usec = 200000;   // 200 ms
        setsockopt(socket_from_car_, SOL_SOCKET, SO_RCVTIMEO,
                   &read_timeout, sizeof(read_timeout));

        sockaddr_in listen_address;
        memset(&listen_address, 0, sizeof(listen_address));
        listen_address.sin_family = AF_INET;
        listen_address.sin_addr.s_addr = htonl(INADDR_ANY);
        listen_address.sin_port = htons((uint16_t)port_from_car_);

        if (bind(socket_from_car_, (sockaddr*)&listen_address, sizeof(listen_address)) < 0) {
            RCLCPP_ERROR(get_logger(), "could not bind port %d: %s",
                         port_from_car_, strerror(errno));
            return false;
        }
        return true;
    }

    void send_command_to_car() {
        DriveCommand command_to_send = latest_command_;

        // if whoever was driving has gone quiet, stop the motor. we leave the
        // steering where it was on purpose, because centring the wheels at the
        // same moment the motor cuts makes the car lurch sideways
        if (got_a_command_ &&
            (now() - last_command_time_).seconds() > command_timeout_sec_) {
            command_to_send.motor_pwm = 0;
            if (!already_warned_about_timeout_) {
                RCLCPP_WARN(get_logger(), "no /cmd_drive for %.1fs, motor held at zero",
                            command_timeout_sec_);
                already_warned_about_timeout_ = true;
            }
        } else if (got_a_command_) {
            already_warned_about_timeout_ = false;
        }

        unsigned char packet[communication::COMMAND_PACKET_BYTES];
        communication::build_command(packet, command_to_send);

        ssize_t bytes_sent = sendto(socket_to_car_, packet, sizeof(packet), 0,
                                    (const sockaddr*)&car_address_, sizeof(car_address_));
        if (bytes_sent < 0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "sendto failed: %s", strerror(errno));
        }
    }

    void receive_from_car_loop() {
        unsigned char packet[64];
        Feedback feedback;

        while (receiver_running_) {
            ssize_t bytes_read = recvfrom(socket_from_car_, packet, sizeof(packet),
                                          0, nullptr, nullptr);
            if (bytes_read <= 0) continue;   // timed out, or we are shutting down

            if (!communication::parse_feedback(packet, (int)bytes_read, feedback)) {
                continue;   // not one of ours
            }

            std_msgs::msg::Int32MultiArray message;
            message.data = {feedback.left_wheel_ticks,
                            feedback.right_wheel_ticks,
                            feedback.forward_speed_cms,
                            feedback.sideways_speed_cms};
            feedback_pub_->publish(message);
        }
    }

    std::string car_ip_;
    int port_to_car_ = 0;
    int port_from_car_ = 0;
    double send_rate_hz_ = 20.0;
    double command_timeout_sec_ = 0.5;

    int socket_to_car_ = -1;
    int socket_from_car_ = -1;
    sockaddr_in car_address_;

    DriveCommand latest_command_;
    rclcpp::Time last_command_time_;
    bool got_a_command_ = false;
    bool already_warned_about_timeout_ = false;

    std::atomic<bool> receiver_running_{false};
    std::thread receiver_thread_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr feedback_pub_;
    rclcpp::TimerBase::SharedPtr send_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<CommunicationNode>());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("communication_node"), "%s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
