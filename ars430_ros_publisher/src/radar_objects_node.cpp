/* radar_objects (ROS2 Jazzy)
 *
 * Turns the radar's internal tracker output (service 230, decoded by
 * radar_publisher onto /radar_objects_raw_<id>) into RViz markers:
 * one person-sized CUBE per tracked object plus a floating text label
 * "ID <n> <prob>%". Colours are stable per track id; the cube opacity is
 * proportional to the track's existence probability.
 *
 * Every filter/appearance knob is a DYNAMIC parameter:
 *   prob_min          existence probability threshold 0..100 (the "is this
 *                     really an object?" knob)
 *   min_seen_scans    persistence: only show tracks seen in at least N scans
 *   range_min / range_max   radial gates [m]
 *   invert_y          flip lateral axis to match the point cloud (calibrate
 *                     by walking: marker must move the same way you do)
 *   marker_size_xy / marker_size_z / marker_alpha / marker_lifetime_s
 *   show_labels       text labels on/off
 */

#include <cmath>
#include <mutex>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "ars430_ros_publisher/msg/radar_object_list.hpp"

namespace {

struct RGB { float r, g, b; };

// Stable, well-separated colour per track id (golden-ratio hue spacing)
RGB colourForId(uint8_t id) {
  const float h6 = std::fmod(id * 0.6180339887f, 1.0f) * 6.0f;
  const float x = 1.0f - std::fabs(std::fmod(h6, 2.0f) - 1.0f);
  RGB c;
  if      (h6 < 1) c = {1, x, 0};
  else if (h6 < 2) c = {x, 1, 0};
  else if (h6 < 3) c = {0, 1, x};
  else if (h6 < 4) c = {0, x, 1};
  else if (h6 < 5) c = {x, 0, 1};
  else             c = {1, 0, x};
  return c;
}

}  // namespace

class RadarObjectsNode : public rclcpp::Node {
public:
  RadarObjectsNode() : Node("radar_objects") {
    const int id = static_cast<int>(declare_parameter<int64_t>("id", 1));

    prob_min_          = declare_parameter<int64_t>("prob_min", 50);
    min_seen_scans_    = declare_parameter<int64_t>("min_seen_scans", 3);
    range_min_         = declare_parameter<double>("range_min", 0.5);
    range_max_         = declare_parameter<double>("range_max", 100.0);
    invert_y_          = declare_parameter<bool>("invert_y", false);
    offset_x_          = declare_parameter<double>("offset_x", 0.0);
    offset_y_          = declare_parameter<double>("offset_y", 0.0);
    marker_size_xy_    = declare_parameter<double>("marker_size_xy", 0.5);
    marker_size_z_     = declare_parameter<double>("marker_size_z", 1.7);
    marker_alpha_      = declare_parameter<double>("marker_alpha", 0.8);
    marker_lifetime_s_ = declare_parameter<double>("marker_lifetime_s", 0.5);
    show_labels_       = declare_parameter<bool>("show_labels", true);

    param_cb_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params) {
          rcl_interfaces::msg::SetParametersResult res;
          res.successful = true;
          std::lock_guard<std::mutex> lk(mtx_);
          for (const auto& p : params) {
            const auto& n = p.get_name();
            if      (n == "prob_min")          prob_min_ = p.as_int();
            else if (n == "min_seen_scans")    min_seen_scans_ = p.as_int();
            else if (n == "range_min")         range_min_ = p.as_double();
            else if (n == "range_max")         range_max_ = p.as_double();
            else if (n == "invert_y")          invert_y_ = p.as_bool();
            else if (n == "offset_x")          offset_x_ = p.as_double();
            else if (n == "offset_y")          offset_y_ = p.as_double();
            else if (n == "marker_size_xy")    marker_size_xy_ = p.as_double();
            else if (n == "marker_size_z")     marker_size_z_ = p.as_double();
            else if (n == "marker_alpha")      marker_alpha_ = p.as_double();
            else if (n == "marker_lifetime_s") marker_lifetime_s_ = p.as_double();
            else if (n == "show_labels")       show_labels_ = p.as_bool();
            else if (n == "id") { res.successful = false; res.reason = "id is fixed at startup"; }
          }
          return res;
        });

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/radar/markers", rclcpp::QoS(10));
    sub_ = create_subscription<ars430_ros_publisher::msg::RadarObjectList>(
        "/radar_objects_raw_" + std::to_string(id), rclcpp::QoS(50),
        [this](ars430_ros_publisher::msg::RadarObjectList::ConstSharedPtr msg) { onScan(*msg); });

    report_timer_ = create_wall_timer(std::chrono::seconds(5), [this] { report(); });
  }

private:
  void onScan(const ars430_ros_publisher::msg::RadarObjectList& scan) {
    int64_t prob_min, min_seen;
    double range_min, range_max, size_xy, size_z, alpha, lifetime, off_x, off_y;
    bool invert_y, show_labels;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      prob_min = prob_min_; min_seen = min_seen_scans_;
      range_min = range_min_; range_max = range_max_;
      invert_y = invert_y_; show_labels = show_labels_;
      off_x = offset_x_; off_y = offset_y_;
      size_xy = marker_size_xy_; size_z = marker_size_z_;
      alpha = marker_alpha_; lifetime = marker_lifetime_s_;
    }

    // Persistence bookkeeping: consecutive-scan counter per track id
    scans_total_++;
    for (auto& [tid, st] : tracks_) st.seen_this_scan = false;
    for (const auto& o : scan.objects) {
      auto& st = tracks_[o.id];
      st.count++;
      st.seen_this_scan = true;
    }
    for (auto it = tracks_.begin(); it != tracks_.end();) {
      if (!it->second.seen_this_scan) {
        it = tracks_.erase(it); // track dropped by the radar: reset persistence
      } else {
        ++it;
      }
    }

    visualization_msgs::msg::MarkerArray arr;
    stat_raw_ += scan.objects.size();
    for (const auto& o : scan.objects) {
      if (o.prob < prob_min) continue;
      if (tracks_[o.id].count < min_seen) continue;
      const double x = o.pos_x + off_x;
      const double y = (invert_y ? -o.pos_y : o.pos_y) + off_y;
      const double range = std::hypot(x, y);
      if (range < range_min || range > range_max) continue;
      stat_shown_++;

      const RGB c = colourForId(o.id);
      const float a = static_cast<float>(alpha) * std::max<float>(o.prob, 20.0f) / 100.0f;

      visualization_msgs::msg::Marker cube;
      cube.header = scan.header;
      cube.ns = "radar_objects";
      cube.id = o.id * 2;
      cube.type = visualization_msgs::msg::Marker::CUBE;
      cube.action = visualization_msgs::msg::Marker::ADD;
      cube.pose.position.x = x;
      cube.pose.position.y = y;
      cube.pose.position.z = size_z / 2.0;
      cube.pose.orientation.w = 1.0;
      cube.scale.x = size_xy;
      cube.scale.y = size_xy;
      cube.scale.z = size_z;
      cube.color.r = c.r; cube.color.g = c.g; cube.color.b = c.b; cube.color.a = a;
      cube.lifetime = rclcpp::Duration::from_seconds(lifetime);
      arr.markers.push_back(cube);

      if (show_labels) {
        visualization_msgs::msg::Marker text;
        text.header = scan.header;
        text.ns = "radar_objects_labels";
        text.id = o.id * 2 + 1;
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::msg::Marker::ADD;
        text.pose.position.x = x;
        text.pose.position.y = y;
        text.pose.position.z = size_z + 0.35;
        text.pose.orientation.w = 1.0;
        text.scale.z = 0.4;
        text.color.r = 1.0f; text.color.g = 1.0f; text.color.b = 1.0f; text.color.a = 1.0f;
        char buf[48];
        snprintf(buf, sizeof(buf), "ID %u  %u%%", o.id, o.prob);
        text.text = buf;
        text.lifetime = rclcpp::Duration::from_seconds(lifetime);
        arr.markers.push_back(text);
      }
    }

    if (!arr.markers.empty()) {
      marker_pub_->publish(arr);
    }
  }

  void report() {
    if (scans_total_ == 0) return;
    RCLCPP_INFO(get_logger(), "last 5s: scans=%lu | objects raw=%lu shown=%lu | active tracks=%zu",
                (unsigned long)scans_total_, (unsigned long)stat_raw_,
                (unsigned long)stat_shown_, tracks_.size());
    scans_total_ = stat_raw_ = stat_shown_ = 0;
  }

  struct TrackState { uint32_t count = 0; bool seen_this_scan = false; };
  std::unordered_map<uint8_t, TrackState> tracks_;

  int64_t prob_min_ = 50, min_seen_scans_ = 3;
  double range_min_ = 0.5, range_max_ = 100.0;
  double offset_x_ = 0.0, offset_y_ = 0.0;
  bool invert_y_ = false, show_labels_ = true;
  double marker_size_xy_ = 0.5, marker_size_z_ = 1.7, marker_alpha_ = 0.8, marker_lifetime_s_ = 0.5;
  std::mutex mtx_;
  uint64_t scans_total_ = 0, stat_raw_ = 0, stat_shown_ = 0;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<ars430_ros_publisher::msg::RadarObjectList>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr report_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RadarObjectsNode>());
  rclcpp::shutdown();
  return 0;
}
