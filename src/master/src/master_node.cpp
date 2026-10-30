// master_node.cpp — objective 7: the node that actually decides what to do.
//
//   subscribes : /vision/turn_target  geometry_msgs/PointStamped
//                /car_feedback        std_msgs/Int32MultiArray
//   publishes  : /cmd_drive           geometry_msgs/Twist
//
// Two modes, switchable at runtime:
//   follow_lane  steer on the vision error
//   goto_point   dead reckon to a world-frame target
//
//     ros2 param set /master_node mode goto_point
//     ros2 param set /master_node target_x 10.0
//
// SIGN WARNING, read this before blaming the controller: if the car swerves
// off the road immediately and consistently, the steering sign is inverted
// for your setup. Do not add gain — negate it:
//     ros2 param set /master_node steer_gain -25.0

#include "master/ackermann.hpp"

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

namespace {

class MasterNode : public rclcpp::Node {
 public:
  MasterNode() : rclcpp::Node("master_node") {
    mode_ = declare_parameter<std::string>("mode", "follow_lane");

    // Positive turns left. Negative because a positive lateral error means
    // the lane centre is to the RIGHT, which needs a right turn.
    steer_gain_ = declare_parameter<double>("steer_gain", -25.0);

    limits_.wheelbase = declare_parameter<double>("wheelbase", master::kDefaultWheelbaseM);
    limits_.steer_limit_deg = declare_parameter<double>("steer_limit_deg", 35.0);
    limits_.max_pwm = declare_parameter<double>("max_pwm", 120.0);
    limits_.min_pwm = declare_parameter<double>("min_pwm", 60.0);
    limits_.arrive_tol_m = declare_parameter<double>("arrive_tol_m", 0.5);

    target_x_ = declare_parameter<double>("target_x", 10.0);
    target_y_ = declare_parameter<double>("target_y", 0.0);

    vision_timeout_s_ = declare_parameter<double>("vision_timeout_s", 0.5);
    rate_hz_ = declare_parameter<double>("rate_hz", 20.0);

    param_cb_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& changed) { return onParams(changed); });

    drive_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_drive", 10);

    target_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
        "/vision/turn_target", 10,
        [this](geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {
          lateral_error_ = msg->point.x;
          vision_valid_ = msg->point.z > 0.5;
          last_vision_time_ = now();
          have_vision_ = true;
        });

    feedback_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
        "/car_feedback", 10,
        [this](std_msgs::msg::Int32MultiArray::ConstSharedPtr msg) {
          if (msg->data.size() >= 3) {
            speed_mps_ = static_cast<double>(msg->data[2]) / 100.0;  // cm/s -> m/s
          }
        });

    const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period), [this] { tick(); });

    last_tick_ = now();
    RCLCPP_INFO(get_logger(), "mode=%s steer_gain=%.1f", mode_.c_str(), steer_gain_);
  }

 private:
  rcl_interfaces::msg::SetParametersResult onParams(
      const std::vector<rclcpp::Parameter>& changed) {
    for (const auto& p : changed) {
      const std::string& n = p.get_name();
      if (n == "mode") {
        mode_ = p.as_string();
        odom_.reset();  // a new mode means a fresh journey
        arrived_logged_ = false;
        RCLCPP_INFO(get_logger(), "mode -> %s", mode_.c_str());
      } else if (n == "steer_gain") steer_gain_ = p.as_double();
      else if (n == "wheelbase") limits_.wheelbase = p.as_double();
      else if (n == "steer_limit_deg") limits_.steer_limit_deg = p.as_double();
      else if (n == "max_pwm") limits_.max_pwm = p.as_double();
      else if (n == "min_pwm") limits_.min_pwm = p.as_double();
      else if (n == "arrive_tol_m") limits_.arrive_tol_m = p.as_double();
      else if (n == "target_x") { target_x_ = p.as_double(); arrived_logged_ = false; }
      else if (n == "target_y") { target_y_ = p.as_double(); arrived_logged_ = false; }
      else if (n == "vision_timeout_s") vision_timeout_s_ = p.as_double();
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  void tick() {
    const rclcpp::Time t = now();
    const double dt = (t - last_tick_).seconds();
    last_tick_ = t;

    // Heading is integrated from what we last commanded, since the car has no
    // IMU. Do this every tick regardless of mode so a switch to goto_point
    // does not start from a stale pose.
    odom_.update(speed_mps_, last_steer_deg_, limits_.wheelbase, dt);

    double pwm = 0.0;
    double steer = last_steer_deg_;

    if (mode_ == "goto_point") {
      double cx = 0.0;
      double cy = 0.0;
      odom_.toCarFrame(target_x_, target_y_, cx, cy);
      const master::Motion m = master::motionToPoint(cx, cy, limits_);
      pwm = m.pwm;
      steer = m.steer_deg;
      if (m.arrived && !arrived_logged_) {
        RCLCPP_INFO(get_logger(), "arrived at (%.1f, %.1f)", target_x_, target_y_);
        arrived_logged_ = true;
      }
    } else {
      // follow_lane
      const bool fresh = have_vision_ && (t - last_vision_time_).seconds() <= vision_timeout_s_;
      if (fresh && vision_valid_) {
        steer = std::clamp(steer_gain_ * lateral_error_, -limits_.steer_limit_deg,
                           limits_.steer_limit_deg);
        pwm = master::speedForSteer(steer, limits_.steer_limit_deg, limits_.max_pwm,
                                    limits_.min_pwm);
      } else {
        // No usable lane. Coast on the last steering angle rather than
        // slamming the wheels straight, which would drive out of a bend.
        pwm = 0.0;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "no usable lane, coasting");
      }
    }

    last_steer_deg_ = steer;

    geometry_msgs::msg::Twist msg;
    msg.linear.x = pwm;
    msg.angular.z = steer;
    drive_pub_->publish(msg);
  }

  std::string mode_;
  double steer_gain_ = -25.0;
  master::MotionLimits limits_{};
  master::Odometry odom_{};

  double target_x_ = 10.0;
  double target_y_ = 0.0;
  bool arrived_logged_ = false;

  double lateral_error_ = 0.0;
  bool vision_valid_ = false;
  bool have_vision_ = false;
  rclcpp::Time last_vision_time_;
  double vision_timeout_s_ = 0.5;

  double speed_mps_ = 0.0;
  double last_steer_deg_ = 0.0;
  double rate_hz_ = 20.0;
  rclcpp::Time last_tick_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr drive_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr feedback_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MasterNode>());
  rclcpp::shutdown();
  return 0;
}
