// master_node.cpp
// the node that decides what the car actually does
//
// listens on  /vision/turn_target  (PointStamped from vision_node)
//             /car_feedback        (Int32MultiArray from communication_node)
// publishes   /cmd_drive           (Twist)
//
// two modes you can swap at runtime:
//   follow_lane   steer on what vision says
//   goto_point    drive to a point, working out where we are as we go
//
//   ros2 param set /master_node mode goto_point
//   ros2 param set /master_node target_x 15.0
//
// if the car swerves off the road straight away and does it every time, the
// steering sign is backwards for your setup. dont add gain, flip it:
//   ros2 param set /master_node steer_gain 25.0

#include "master/ackermann.hpp"

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

#include <string>
#include <vector>

class MasterNode : public rclcpp::Node {
public:
    MasterNode() : rclcpp::Node("master_node") {
        mode_ = declare_parameter<std::string>("mode", "follow_lane");

        // negative because a positive offset means the lane centre is to our
        // RIGHT, and turning right is a negative steering angle
        steering_gain_ = declare_parameter<double>("steer_gain", -25.0);

        limits_.wheelbase_m = declare_parameter<double>("wheelbase", master::DEFAULT_WHEELBASE_M);
        limits_.max_steering_deg = declare_parameter<double>("steer_limit_deg", 35.0);
        limits_.max_pwm = declare_parameter<double>("max_pwm", 120.0);
        limits_.min_pwm = declare_parameter<double>("min_pwm", 60.0);
        limits_.arrived_distance_m = declare_parameter<double>("arrive_tol_m", 0.5);

        target_x_ = declare_parameter<double>("target_x", 10.0);
        target_y_ = declare_parameter<double>("target_y", 0.0);

        vision_timeout_sec_ = declare_parameter<double>("vision_timeout_s", 0.5);
        control_rate_hz_ = declare_parameter<double>("rate_hz", 20.0);

        settings_callback_ = add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& changed) {
                return on_settings_changed(changed);
            });

        drive_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_drive", 10);

        turn_target_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
            "/vision/turn_target", 10,
            [this](geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {
                lane_offset_ = msg->point.x;
                lane_is_usable_ = msg->point.z > 0.5;
                last_vision_time_ = now();
                got_vision_ = true;
            });

        feedback_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
            "/car_feedback", 10,
            [this](std_msgs::msg::Int32MultiArray::ConstSharedPtr msg) {
                if (msg->data.size() >= 3) {
                    measured_speed_mps_ = msg->data[2] / 100.0;   // cm/s to m/s
                }
            });

        auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
        control_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            [this] { control_tick(); });

        last_tick_time_ = now();
        RCLCPP_INFO(get_logger(), "mode %s, steer gain %.1f", mode_.c_str(), steering_gain_);
    }

private:
    rcl_interfaces::msg::SetParametersResult on_settings_changed(
        const std::vector<rclcpp::Parameter>& changed) {
        for (const auto& parameter : changed) {
            const std::string& name = parameter.get_name();
            if (name == "mode") {
                mode_ = parameter.as_string();
                odometry_.reset();            // new mode means a fresh journey
                already_said_arrived_ = false;
                RCLCPP_INFO(get_logger(), "mode is now %s", mode_.c_str());
            }
            else if (name == "steer_gain")      steering_gain_ = parameter.as_double();
            else if (name == "wheelbase")       limits_.wheelbase_m = parameter.as_double();
            else if (name == "steer_limit_deg") limits_.max_steering_deg = parameter.as_double();
            else if (name == "max_pwm")         limits_.max_pwm = parameter.as_double();
            else if (name == "min_pwm")         limits_.min_pwm = parameter.as_double();
            else if (name == "arrive_tol_m")    limits_.arrived_distance_m = parameter.as_double();
            else if (name == "target_x")      { target_x_ = parameter.as_double(); already_said_arrived_ = false; }
            else if (name == "target_y")      { target_y_ = parameter.as_double(); already_said_arrived_ = false; }
            else if (name == "vision_timeout_s") vision_timeout_sec_ = parameter.as_double();
        }

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }

    void control_tick() {
        rclcpp::Time this_tick = now();
        double seconds_since_last = (this_tick - last_tick_time_).seconds();
        last_tick_time_ = this_tick;

        // update where we think we are every tick whatever the mode, so that
        // switching to goto_point doesnt start from a stale position
        odometry_.update(measured_speed_mps_, last_steering_deg_,
                         limits_.wheelbase_m, seconds_since_last);

        double pwm = 0.0;
        double steering_deg = last_steering_deg_;

        if (mode_ == "goto_point") {
            double target_ahead = 0.0;
            double target_left = 0.0;
            odometry_.to_car_frame(target_x_, target_y_, target_ahead, target_left);

            master::DriveDecision decision =
                master::drive_towards(target_ahead, target_left, limits_);

            pwm = decision.pwm;
            steering_deg = decision.steering_deg;

            if (decision.arrived && !already_said_arrived_) {
                RCLCPP_INFO(get_logger(), "arrived at (%.1f, %.1f)", target_x_, target_y_);
                already_said_arrived_ = true;
            }
        } else {
            bool vision_is_fresh = got_vision_ &&
                (this_tick - last_vision_time_).seconds() <= vision_timeout_sec_;

            if (vision_is_fresh && lane_is_usable_) {
                steering_deg = steering_gain_ * lane_offset_;
                if (steering_deg >  limits_.max_steering_deg) steering_deg =  limits_.max_steering_deg;
                if (steering_deg < -limits_.max_steering_deg) steering_deg = -limits_.max_steering_deg;

                pwm = master::throttle_for_steering(steering_deg, limits_.max_steering_deg,
                                                    limits_.max_pwm, limits_.min_pwm);
            } else {
                // no usable lane. coast on the steering we already had rather
                // than snapping the wheels straight, which would drive us out
                // of whatever bend we are in
                pwm = 0.0;
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                     "no usable lane, coasting");
            }
        }

        last_steering_deg_ = steering_deg;

        geometry_msgs::msg::Twist command;
        command.linear.x = pwm;
        command.angular.z = steering_deg;
        drive_pub_->publish(command);
    }

    std::string mode_;
    double steering_gain_ = -25.0;
    master::DrivingLimits limits_;
    master::Odometry odometry_;

    double target_x_ = 10.0;
    double target_y_ = 0.0;
    bool already_said_arrived_ = false;

    double lane_offset_ = 0.0;
    bool lane_is_usable_ = false;
    bool got_vision_ = false;
    rclcpp::Time last_vision_time_;
    double vision_timeout_sec_ = 0.5;

    double measured_speed_mps_ = 0.0;
    double last_steering_deg_ = 0.0;
    double control_rate_hz_ = 20.0;
    rclcpp::Time last_tick_time_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr drive_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr turn_target_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr feedback_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr settings_callback_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MasterNode>());
    rclcpp::shutdown();
    return 0;
}
