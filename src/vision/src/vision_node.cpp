// vision_node.cpp
// wraps LaneFinder in a ROS node
//
// listens on  the camera topic       (sensor_msgs/Image, bgr8)
// publishes   /vision/turn_target    (PointStamped: x = how far off centre, z = 1 if usable)
//             /vision/debug_image    (Image, the birds eye view with our decision drawn on)
//
// every threshold is a ros parameter so you can tune it with ros2 param set
// while the sim runs, instead of rebuild, restart, look, repeat

#include "vision/lane_finder.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <string>
#include <vector>

class VisionNode : public rclcpp::Node {
public:
    VisionNode() : rclcpp::Node("vision_node") {
        camera_topic_ = declare_parameter<std::string>(
            "image_topic", "/catvehicle/camera_front/image_raw_front");
        publish_debug_image_ = declare_parameter<bool>("publish_debug", true);

        vision::LaneSettings settings;
        settings.min_lightness = declare_parameter<int>("min_lightness", settings.min_lightness);
        settings.max_saturation = declare_parameter<int>("max_saturation", settings.max_saturation);
        settings.despeckle_passes = declare_parameter<int>("despeckle_passes", settings.despeckle_passes);
        settings.horizon_row = declare_parameter<double>("horizon_row", settings.horizon_row);
        settings.bottom_row = declare_parameter<double>("bottom_row", settings.bottom_row);
        settings.half_width_at_horizon = declare_parameter<double>("half_width_at_horizon", settings.half_width_at_horizon);
        settings.half_width_at_bottom = declare_parameter<double>("half_width_at_bottom", settings.half_width_at_bottom);
        settings.warped_size_px = declare_parameter<int>("warped_size_px", settings.warped_size_px);
        settings.band_far_edge = declare_parameter<double>("band_far_edge", settings.band_far_edge);
        settings.band_near_edge = declare_parameter<double>("band_near_edge", settings.band_near_edge);
        settings.min_white_per_column = declare_parameter<int>("min_white_per_column", settings.min_white_per_column);
        settings.min_marking_width_px = declare_parameter<int>("min_marking_width_px", settings.min_marking_width_px);
        settings.join_gap_px = declare_parameter<int>("join_gap_px", settings.join_gap_px);
        settings.max_marking_width_px = declare_parameter<int>("max_marking_width_px", settings.max_marking_width_px);
        settings.min_lane_width_fraction = declare_parameter<double>("min_lane_width_fraction", settings.min_lane_width_fraction);
        lane_finder_.settings() = settings;

        settings_callback_ = add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& changed) {
                return on_settings_changed(changed);
            });

        turn_target_pub_ =
            create_publisher<geometry_msgs::msg::PointStamped>("/vision/turn_target", 10);
        if (publish_debug_image_) {
            debug_image_pub_ = create_publisher<sensor_msgs::msg::Image>("/vision/debug_image", 1);
        }

        // sensor data QoS: best effort, small queue. the reliable default
        // fights a camera publisher and shows up as latency that keeps growing
        camera_sub_ = create_subscription<sensor_msgs::msg::Image>(
            camera_topic_, rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { on_camera_frame(msg); });

        RCLCPP_INFO(get_logger(), "watching %s", camera_topic_.c_str());
    }

private:
    rcl_interfaces::msg::SetParametersResult on_settings_changed(
        const std::vector<rclcpp::Parameter>& changed) {
        vision::LaneSettings& settings = lane_finder_.settings();

        for (const auto& parameter : changed) {
            const std::string& name = parameter.get_name();
            if      (name == "min_lightness")        settings.min_lightness = parameter.as_int();
            else if (name == "max_saturation")       settings.max_saturation = parameter.as_int();
            else if (name == "despeckle_passes")     settings.despeckle_passes = parameter.as_int();
            else if (name == "horizon_row")          settings.horizon_row = parameter.as_double();
            else if (name == "bottom_row")           settings.bottom_row = parameter.as_double();
            else if (name == "half_width_at_horizon") settings.half_width_at_horizon = parameter.as_double();
            else if (name == "half_width_at_bottom") settings.half_width_at_bottom = parameter.as_double();
            else if (name == "band_far_edge")        settings.band_far_edge = parameter.as_double();
            else if (name == "band_near_edge")       settings.band_near_edge = parameter.as_double();
            else if (name == "min_white_per_column") settings.min_white_per_column = parameter.as_int();
            else if (name == "min_marking_width_px") settings.min_marking_width_px = parameter.as_int();
            else if (name == "join_gap_px")          settings.join_gap_px = parameter.as_int();
            else if (name == "max_marking_width_px") settings.max_marking_width_px = parameter.as_int();
            else if (name == "min_lane_width_fraction") settings.min_lane_width_fraction = parameter.as_double();
        }

        // the geometry just moved, so the lane width we remembered no longer
        // means the same thing
        lane_finder_.reset();

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }

    void on_camera_frame(const sensor_msgs::msg::Image::ConstSharedPtr& message) {
        cv::Mat camera_frame;
        try {
            camera_frame = cv_bridge::toCvShare(message, "bgr8")->image;
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "cv_bridge: %s", e.what());
            return;
        }
        if (camera_frame.empty()) return;

        vision::LaneResult result = lane_finder_.process(camera_frame);

        // a long run of useless frames means we are somewhere the remembered
        // lane width no longer describes, so stop trusting it
        if (result.usable) {
            bad_frames_in_a_row_ = 0;
        } else if (++bad_frames_in_a_row_ > 30) {
            lane_finder_.reset();
            bad_frames_in_a_row_ = 0;
        }

        geometry_msgs::msg::PointStamped target;
        target.header = message->header;
        target.point.x = result.offset_from_centre;
        target.point.y = 0.0;
        target.point.z = result.usable ? 1.0 : 0.0;
        turn_target_pub_->publish(target);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "%s  offset %+.2f",
                             vision::lane_source_name(result.source),
                             result.offset_from_centre);

        if (debug_image_pub_ && debug_image_pub_->get_subscription_count() > 0) {
            cv::Mat picture = lane_finder_.debug_picture(result);
            if (!picture.empty()) {
                debug_image_pub_->publish(
                    *cv_bridge::CvImage(message->header, "bgr8", picture).toImageMsg());
            }
        }
    }

    std::string camera_topic_;
    bool publish_debug_image_ = true;
    int bad_frames_in_a_row_ = 0;

    vision::LaneFinder lane_finder_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr turn_target_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr settings_callback_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionNode>());
    rclcpp::shutdown();
    return 0;
}
