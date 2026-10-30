// vision_node.cpp — wraps LaneFinder in a ROS node.
//
//   subscribes : <image_topic>          sensor_msgs/Image  (camera, BGR8)
//   publishes  : /vision/turn_target    geometry_msgs/PointStamped
//                                         x = lateral error, -1 .. +1
//                                         z = 1.0 when usable, 0.0 when not
//                /vision/debug_image    sensor_msgs/Image
//
// Every threshold is a ROS parameter so the whole thing can be tuned with
// `ros2 param set` while the sim runs, instead of rebuild-restart-look.

#include "vision/lane_finder.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <memory>
#include <string>

namespace {

class VisionNode : public rclcpp::Node {
 public:
  VisionNode() : rclcpp::Node("vision_node") {
    image_topic_ = declare_parameter<std::string>(
        "image_topic", "/catvehicle/camera_front/image_raw_front");
    publish_debug_ = declare_parameter<bool>("publish_debug", true);

    vision::LaneParams p;
    p.l_min = declare_parameter<int>("l_min", p.l_min);
    p.s_max = declare_parameter<int>("s_max", p.s_max);
    p.morph_open = declare_parameter<int>("morph_open", p.morph_open);
    p.horizon_y = declare_parameter<double>("horizon_y", p.horizon_y);
    p.bottom_y = declare_parameter<double>("bottom_y", p.bottom_y);
    p.top_half_width = declare_parameter<double>("top_half_width", p.top_half_width);
    p.bottom_half_width = declare_parameter<double>("bottom_half_width", p.bottom_half_width);
    p.bev_size = declare_parameter<int>("bev_size", p.bev_size);
    p.band_far = declare_parameter<double>("band_far", p.band_far);
    p.band_near = declare_parameter<double>("band_near", p.band_near);
    p.min_column_hits = declare_parameter<int>("min_column_hits", p.min_column_hits);
    p.min_run_px = declare_parameter<int>("min_run_px", p.min_run_px);
    p.merge_gap_px = declare_parameter<int>("merge_gap_px", p.merge_gap_px);
    p.max_line_run_px = declare_parameter<int>("max_line_run_px", p.max_line_run_px);
    p.min_lane_frac = declare_parameter<double>("min_lane_frac", p.min_lane_frac);
    finder_.params() = p;

    // Parameters are re-read on every change so tuning is live.
    param_cb_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& changed) {
          return onParams(changed);
        });

    target_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/vision/turn_target", 10);
    if (publish_debug_) {
      debug_pub_ = create_publisher<sensor_msgs::msg::Image>("/vision/debug_image", 1);
    }

    // Sensor-data QoS: best effort, small queue. The reliable default fights
    // a camera publisher and shows up as growing latency.
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { onImage(msg); });

    RCLCPP_INFO(get_logger(), "subscribed to %s", image_topic_.c_str());
  }

 private:
  rcl_interfaces::msg::SetParametersResult onParams(
      const std::vector<rclcpp::Parameter>& changed) {
    vision::LaneParams& p = finder_.params();
    for (const auto& param : changed) {
      const std::string& n = param.get_name();
      if (n == "l_min") p.l_min = param.as_int();
      else if (n == "s_max") p.s_max = param.as_int();
      else if (n == "morph_open") p.morph_open = param.as_int();
      else if (n == "horizon_y") p.horizon_y = param.as_double();
      else if (n == "bottom_y") p.bottom_y = param.as_double();
      else if (n == "top_half_width") p.top_half_width = param.as_double();
      else if (n == "bottom_half_width") p.bottom_half_width = param.as_double();
      else if (n == "band_far") p.band_far = param.as_double();
      else if (n == "band_near") p.band_near = param.as_double();
      else if (n == "min_column_hits") p.min_column_hits = param.as_int();
      else if (n == "min_run_px") p.min_run_px = param.as_int();
      else if (n == "merge_gap_px") p.merge_gap_px = param.as_int();
      else if (n == "max_line_run_px") p.max_line_run_px = param.as_int();
      else if (n == "min_lane_frac") p.min_lane_frac = param.as_double();
    }
    // Geometry changed, so the remembered lane width is no longer comparable.
    finder_.reset();

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    cv::Mat frame;
    try {
      frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "cv_bridge: %s", e.what());
      return;
    }
    if (frame.empty()) {
      return;
    }

    const vision::LaneResult r = finder_.process(frame);

    // Long runs of unusable frames mean the car is somewhere the remembered
    // width no longer describes, so stop trusting it.
    if (r.found) {
      lost_frames_ = 0;
    } else if (++lost_frames_ > 30) {
      finder_.reset();
      lost_frames_ = 0;
    }

    geometry_msgs::msg::PointStamped out;
    out.header = msg->header;
    out.point.x = r.lateral_error;
    out.point.y = 0.0;
    out.point.z = r.found ? 1.0 : 0.0;
    target_pub_->publish(out);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "%s err %+.2f",
                         vision::toString(r.source), r.lateral_error);

    if (debug_pub_ && debug_pub_->get_subscription_count() > 0) {
      const cv::Mat dbg = finder_.debugImage(r);
      if (!dbg.empty()) {
        debug_pub_->publish(*cv_bridge::CvImage(msg->header, "bgr8", dbg).toImageMsg());
      }
    }
  }

  std::string image_topic_;
  bool publish_debug_ = true;
  int lost_frames_ = 0;

  vision::LaneFinder finder_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisionNode>());
  rclcpp::shutdown();
  return 0;
}
