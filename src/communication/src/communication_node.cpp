// communication_node.cpp — the only node that touches the UDP socket.
//
// Objectives 1 and 2 of the final project live here: send drive commands to
// the simulator, and read its feedback back into ROS.
//
//   subscribes : /cmd_drive     geometry_msgs/Twist
//                                 linear.x  = motor PWM   (-255 .. 255)
//                                 angular.z = steering    ( -90 ..  90 deg)
//   publishes  : /car_feedback  std_msgs/Int32MultiArray
//                                 [left_ticks, right_ticks, vel_x_cms, vel_y_cms]
//
// Why one socket node instead of letting every node send its own packets:
// two nodes writing to the same port fight each other, and the car twitches.
// Everything funnels through /cmd_drive so there is exactly one writer.

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
#include <cerrno>
#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <thread>

namespace {

using communication::DriveCommand;
using communication::Feedback;

class CommunicationNode : public rclcpp::Node {
 public:
  CommunicationNode() : rclcpp::Node("communication_node") {
    remote_host_ = declare_parameter<std::string>("remote_host", "127.0.0.1");
    send_port_ = declare_parameter<int>("send_port", 12151);
    recv_port_ = declare_parameter<int>("recv_port", 12152);
    rate_hz_ = declare_parameter<double>("rate_hz", 20.0);
    command_timeout_s_ = declare_parameter<double>("command_timeout_s", 0.5);

    if (!openSockets()) {
      throw std::runtime_error("could not open UDP sockets");
    }

    command_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_drive", 10,
        [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) { onCommand(*msg); });

    feedback_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>("/car_feedback", 10);

    const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
    send_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period), [this] { sendTick(); });

    running_ = true;
    receiver_ = std::thread([this] { receiveLoop(); });

    RCLCPP_INFO(get_logger(), "sending to %s:%d, listening on :%d, %.1f Hz",
                remote_host_.c_str(), send_port_, recv_port_, rate_hz_);
  }

  ~CommunicationNode() override {
    // Order matters. Flip the flag, then close the socket so a blocked
    // recvfrom() returns immediately, then join. Joining first would hang
    // for up to one socket timeout.
    running_ = false;
    if (recv_fd_ >= 0) {
      ::shutdown(recv_fd_, SHUT_RDWR);
      ::close(recv_fd_);
      recv_fd_ = -1;
    }
    if (receiver_.joinable()) {
      receiver_.join();
    }
    if (send_fd_ >= 0) {
      ::close(send_fd_);
      send_fd_ = -1;
    }
  }

 private:
  bool openSockets() {
    send_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (send_fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "send socket: %s", std::strerror(errno));
      return false;
    }

    std::memset(&send_addr_, 0, sizeof(send_addr_));
    send_addr_.sin_family = AF_INET;
    send_addr_.sin_port = htons(static_cast<uint16_t>(send_port_));
    if (::inet_pton(AF_INET, remote_host_.c_str(), &send_addr_.sin_addr) != 1) {
      RCLCPP_ERROR(get_logger(), "bad remote_host: %s", remote_host_.c_str());
      return false;
    }

    recv_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (recv_fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "recv socket: %s", std::strerror(errno));
      return false;
    }

    int reuse = 1;
    ::setsockopt(recv_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    // Without a read timeout the receive thread blocks forever and shutdown
    // relies entirely on close(). The timeout is belt and braces.
    timeval tv{};
    tv.tv_sec = 0;
    tv.tv_usec = 200000;  // 200 ms
    ::setsockopt(recv_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    sockaddr_in bind_addr{};
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    bind_addr.sin_port = htons(static_cast<uint16_t>(recv_port_));
    if (::bind(recv_fd_, reinterpret_cast<sockaddr*>(&bind_addr), sizeof(bind_addr)) < 0) {
      RCLCPP_ERROR(get_logger(), "bind :%d failed: %s", recv_port_, std::strerror(errno));
      return false;
    }
    return true;
  }

  void onCommand(const geometry_msgs::msg::Twist& msg) {
    command_.motor_pwm = static_cast<int32_t>(std::lround(msg.linear.x));
    command_.steering_deg = static_cast<int32_t>(std::lround(msg.angular.z));
    last_command_time_ = now();
    have_command_ = true;
  }

  void sendTick() {
    DriveCommand outgoing = command_;

    // If the controller died or the teleop was closed, stop driving. Steering
    // is deliberately left where it was: centring the wheels at the same
    // moment the motor cuts makes the car lurch sideways on the simulator.
    if (have_command_ && (now() - last_command_time_).seconds() > command_timeout_s_) {
      outgoing.motor_pwm = 0;
      if (!timeout_warned_) {
        RCLCPP_WARN(get_logger(), "no /cmd_drive for %.1fs, holding motor at zero",
                    command_timeout_s_);
        timeout_warned_ = true;
      }
    } else if (have_command_) {
      timeout_warned_ = false;
    }

    unsigned char buf[communication::kCommandPacketSize];
    const std::size_t len = communication::buildCommandPacket(buf, outgoing);

    const ssize_t sent = ::sendto(send_fd_, buf, len, 0,
                                  reinterpret_cast<const sockaddr*>(&send_addr_),
                                  sizeof(send_addr_));
    if (sent < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "sendto: %s",
                           std::strerror(errno));
    }
  }

  void receiveLoop() {
    unsigned char buf[64];
    Feedback fb;

    while (running_) {
      const ssize_t len = ::recvfrom(recv_fd_, buf, sizeof(buf), 0, nullptr, nullptr);
      if (len <= 0) {
        continue;  // timeout, or the socket was closed on shutdown
      }
      if (!communication::parseFeedbackPacket(buf, static_cast<std::size_t>(len), fb)) {
        continue;  // not ours
      }

      std_msgs::msg::Int32MultiArray msg;
      msg.data = {fb.left_ticks, fb.right_ticks, fb.vel_x_cms, fb.vel_y_cms};
      feedback_pub_->publish(msg);
    }
  }

  std::string remote_host_;
  int send_port_ = 0;
  int recv_port_ = 0;
  double rate_hz_ = 20.0;
  double command_timeout_s_ = 0.5;

  int send_fd_ = -1;
  int recv_fd_ = -1;
  sockaddr_in send_addr_{};

  DriveCommand command_{};
  rclcpp::Time last_command_time_;
  bool have_command_ = false;
  bool timeout_warned_ = false;

  std::atomic<bool> running_{false};
  std::thread receiver_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr feedback_pub_;
  rclcpp::TimerBase::SharedPtr send_timer_;
};

}  // namespace

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
