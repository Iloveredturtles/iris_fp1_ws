// wasd_teleop_node.cpp
// drive the car from the keyboard
//
//   w / s   throttle forward / back
//   a / d   steer left / right
//   space   motor to zero
//   x       wheels straight
//   q       quit
//
// publishes /cmd_drive, which communication_node turns into packets. this node
// never touches a socket itself
//
// the terminal goes into raw mode so a keypress arrives straight away instead
// of waiting for enter. raw mode belongs to the terminal, not to us, so if we
// exit without putting it back the users shell is left broken with no echo.
// RawTerminal puts it back from its destructor, which runs however we exit

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include <termios.h>
#include <unistd.h>

#include <cstdio>

const double PWM_STEP = 20.0;
const double MAX_PWM = 255.0;
const double STEERING_STEP = 6.0;
const double MAX_STEERING_DEG = 45.0;   // the car accepts 90 but is unstable long before that

class RawTerminal {
public:
    RawTerminal() {
        if (tcgetattr(STDIN_FILENO, &original_settings_) == 0) {
            have_original_ = true;

            termios raw = original_settings_;
            raw.c_lflag &= ~(ICANON | ECHO);
            raw.c_cc[VMIN] = 0;    // read returns straight away
            raw.c_cc[VTIME] = 1;   // after at most 100 ms
            tcsetattr(STDIN_FILENO, TCSANOW, &raw);
        }
    }

    ~RawTerminal() {
        if (have_original_) {
            tcsetattr(STDIN_FILENO, TCSANOW, &original_settings_);
        }
    }

    RawTerminal(const RawTerminal&) = delete;
    RawTerminal& operator=(const RawTerminal&) = delete;

private:
    termios original_settings_;
    bool have_original_ = false;
};

double clamp_double(double value, double lowest, double highest) {
    if (value < lowest)  return lowest;
    if (value > highest) return highest;
    return value;
}

class WasdTeleopNode : public rclcpp::Node {
public:
    WasdTeleopNode() : rclcpp::Node("wasd_teleop_node") {
        command_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_drive", 10);

        // 20 Hz, same as communication_node. we keep republishing the held
        // values so its watchdog doesnt cut the motor while a key is held
        publish_timer_ = create_wall_timer(std::chrono::milliseconds(50),
                                           [this] { publish_tick(); });

        printf("\nwasd teleop\n"
               "  w/s throttle   a/d steer   space stop   x straighten   q quit\n\n");
    }

private:
    void publish_tick() {
        char key = 0;
        if (read(STDIN_FILENO, &key, 1) == 1) {
            handle_key(key);
        }

        geometry_msgs::msg::Twist message;
        message.linear.x = motor_pwm_;
        message.angular.z = steering_deg_;
        command_pub_->publish(message);

        printf("\rpwm %+6.0f   steering %+6.1f deg   ", motor_pwm_, steering_deg_);
        fflush(stdout);
    }

    void handle_key(char key) {
        switch (key) {
            case 'w': case 'W':
                motor_pwm_ = clamp_double(motor_pwm_ + PWM_STEP, -MAX_PWM, MAX_PWM);
                break;
            case 's': case 'S':
                motor_pwm_ = clamp_double(motor_pwm_ - PWM_STEP, -MAX_PWM, MAX_PWM);
                break;
            case 'a': case 'A':
                steering_deg_ = clamp_double(steering_deg_ + STEERING_STEP,
                                             -MAX_STEERING_DEG, MAX_STEERING_DEG);
                break;
            case 'd': case 'D':
                steering_deg_ = clamp_double(steering_deg_ - STEERING_STEP,
                                             -MAX_STEERING_DEG, MAX_STEERING_DEG);
                break;
            case ' ':
                motor_pwm_ = 0.0;
                break;
            case 'x': case 'X':
                steering_deg_ = 0.0;
                break;
            case 'q': case 'Q': case 3: case 27:
                motor_pwm_ = 0.0;
                steering_deg_ = 0.0;
                command_pub_->publish(geometry_msgs::msg::Twist());   // all zeros
                rclcpp::shutdown();
                break;
            default:
                break;
        }
    }

    double motor_pwm_ = 0.0;
    double steering_deg_ = 0.0;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    {
        RawTerminal raw_mode;   // put back when this scope ends, however it ends
        rclcpp::spin(std::make_shared<WasdTeleopNode>());
    }
    rclcpp::shutdown();
    printf("\n");
    return 0;
}
