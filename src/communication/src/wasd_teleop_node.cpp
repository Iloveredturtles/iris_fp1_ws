// wasd_teleop_node.cpp — objective 3: drive the car from the keyboard.
//
// Publishes geometry_msgs/Twist on /cmd_drive, which communication_node turns
// into UDP packets. This node never touches a socket.
//
//   W / S   throttle forward / back
//   A / D   steer left / right
//   SPACE   motor to zero
//   X       wheels straight
//   Q       quit
//
// The terminal is put in raw mode so a keypress arrives immediately instead of
// waiting for Enter. Raw mode is a global property of the terminal, so if this
// node exits without restoring it the user's shell is left broken — no echo,
// no Ctrl-C. RawTerminal restores it from its destructor, which runs on normal
// exit, on an exception, and on a caught SIGINT.

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include <termios.h>
#include <unistd.h>

#include <csignal>
#include <cstdio>
#include <memory>

namespace {

constexpr double kPwmStep = 20.0;
constexpr double kPwmLimit = 255.0;
constexpr double kSteerStep = 6.0;
constexpr double kSteerLimit = 45.0;  // the sim accepts 90, but 45 is already
                                      // far past where the car stays stable

class RawTerminal {
 public:
  RawTerminal() {
    if (::tcgetattr(STDIN_FILENO, &saved_) == 0) {
      saved_valid_ = true;
      termios raw = saved_;
      raw.c_lflag &= ~(ICANON | ECHO);
      raw.c_cc[VMIN] = 0;   // read() returns straight away...
      raw.c_cc[VTIME] = 1;  // ...after at most 100 ms
      ::tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    }
  }

  ~RawTerminal() { restore(); }

  void restore() {
    if (saved_valid_) {
      ::tcsetattr(STDIN_FILENO, TCSANOW, &saved_);
      saved_valid_ = false;
    }
  }

  RawTerminal(const RawTerminal&) = delete;
  RawTerminal& operator=(const RawTerminal&) = delete;

 private:
  termios saved_{};
  bool saved_valid_ = false;
};

double clampd(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

class WasdTeleopNode : public rclcpp::Node {
 public:
  WasdTeleopNode() : rclcpp::Node("wasd_teleop_node") {
    publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_drive", 10);

    // 20 Hz, matching communication_node. Republishing the held values keeps
    // the command fresh so the watchdog there does not cut the motor while a
    // key is simply being held down.
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] { tick(); });

    std::printf(
        "\nWASD teleop\n"
        "  W/S throttle   A/D steer   SPACE stop motor   X centre wheels   Q quit\n\n");
  }

  void tick() {
    char c = 0;
    const ssize_t n = ::read(STDIN_FILENO, &c, 1);
    if (n == 1) {
      apply(c);
    }

    geometry_msgs::msg::Twist msg;
    msg.linear.x = pwm_;
    msg.angular.z = steer_;
    publisher_->publish(msg);

    std::printf("\rpwm %+6.0f   steer %+6.1f deg   ", pwm_, steer_);
    std::fflush(stdout);
  }

 private:
  void apply(char c) {
    switch (c) {
      case 'w': case 'W': pwm_ = clampd(pwm_ + kPwmStep, -kPwmLimit, kPwmLimit); break;
      case 's': case 'S': pwm_ = clampd(pwm_ - kPwmStep, -kPwmLimit, kPwmLimit); break;
      case 'a': case 'A': steer_ = clampd(steer_ - kSteerStep, -kSteerLimit, kSteerLimit); break;
      case 'd': case 'D': steer_ = clampd(steer_ + kSteerStep, -kSteerLimit, kSteerLimit); break;
      case ' ': pwm_ = 0.0; break;
      case 'x': case 'X': steer_ = 0.0; break;
      case 'q': case 'Q': case 3: case 27:
        pwm_ = 0.0;
        steer_ = 0.0;
        publishStop();
        rclcpp::shutdown();
        break;
      default: break;
    }
  }

  void publishStop() {
    geometry_msgs::msg::Twist msg;  // all zeros
    publisher_->publish(msg);
  }

  double pwm_ = 0.0;
  double steer_ = 0.0;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  {
    RawTerminal raw;  // restored when this scope ends, however it ends
    rclcpp::spin(std::make_shared<WasdTeleopNode>());
  }
  rclcpp::shutdown();
  std::printf("\n");
  return 0;
}
