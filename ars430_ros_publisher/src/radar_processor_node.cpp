/* radar_processor (ROS2 Jazzy)
 *
 * Subscribes to /unfiltered_radar_packet_<id>, applies the detection filters
 * and republishes to /filtered_radar_packet_<id>.
 *
 * Every filter threshold is a DYNAMIC parameter: change it live with
 *   ros2 param set /radar_processor velocity_min 0.4
 * and the next packet already uses the new value. See the package README
 * for the full table and recommended values per test scenario.
 */

#include <cmath>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "ars430_ros_publisher/msg/radar_packet.hpp"
#include "ars430_ros_publisher/msg/radar_detection.hpp"

static constexpr uint8_t EVENT_FAR = 2;

struct Filters {
  bool   raw            = false;  // true = forward everything untouched
  double snr_min_near   = 0.0;    // dB
  double snr_min_far    = 0.0;    // dB
  double velocity_min   = 0.0;    // m/s, |vrel| below this is dropped (0 = off)
  double velocity_max   = 100.0;  // m/s
  double range_min      = 0.25;   // m
  double range_max      = 100.0;  // m
  double az_max_deg     = 90.0;   // deg, |azimuth| above this is dropped
  double rcs_min        = -100.0; // dBsm
};

class RadarProcessorNode : public rclcpp::Node {
public:
  RadarProcessorNode() : Node("radar_processor") {
    const int id = static_cast<int>(declare_parameter<int64_t>("id", 1));

    f_.raw          = declare_parameter<bool>("raw", f_.raw);
    f_.snr_min_near = declare_parameter<double>("snr_min_near", f_.snr_min_near);
    f_.snr_min_far  = declare_parameter<double>("snr_min_far", f_.snr_min_far);
    f_.velocity_min = declare_parameter<double>("velocity_min", f_.velocity_min);
    f_.velocity_max = declare_parameter<double>("velocity_max", f_.velocity_max);
    f_.range_min    = declare_parameter<double>("range_min", f_.range_min);
    f_.range_max    = declare_parameter<double>("range_max", f_.range_max);
    f_.az_max_deg   = declare_parameter<double>("az_max_deg", f_.az_max_deg);
    f_.rcs_min      = declare_parameter<double>("rcs_min", f_.rcs_min);

    param_cb_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params) {
          rcl_interfaces::msg::SetParametersResult res;
          res.successful = true;
          std::lock_guard<std::mutex> lk(mtx_);
          for (const auto& p : params) {
            const auto& n = p.get_name();
            if      (n == "raw")           f_.raw = p.as_bool();
            else if (n == "snr_min_near")  f_.snr_min_near = p.as_double();
            else if (n == "snr_min_far")   f_.snr_min_far = p.as_double();
            else if (n == "velocity_min")  f_.velocity_min = p.as_double();
            else if (n == "velocity_max")  f_.velocity_max = p.as_double();
            else if (n == "range_min")     f_.range_min = p.as_double();
            else if (n == "range_max")     f_.range_max = p.as_double();
            else if (n == "az_max_deg")    f_.az_max_deg = p.as_double();
            else if (n == "rcs_min")       f_.rcs_min = p.as_double();
            else if (n == "id") { res.successful = false; res.reason = "id is fixed at startup"; }
            RCLCPP_INFO(get_logger(), "param %s updated", n.c_str());
          }
          return res;
        });

    pub_ = create_publisher<ars430_ros_publisher::msg::RadarPacket>(
        "/filtered_radar_packet_" + std::to_string(id), rclcpp::QoS(50));
    sub_ = create_subscription<ars430_ros_publisher::msg::RadarPacket>(
        "/unfiltered_radar_packet_" + std::to_string(id), rclcpp::QoS(50),
        [this](ars430_ros_publisher::msg::RadarPacket::ConstSharedPtr msg) { process(*msg); });

    report_timer_ = create_wall_timer(std::chrono::seconds(5), [this] { report(); });
  }

private:
  void process(const ars430_ros_publisher::msg::RadarPacket& in) {
    Filters f;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      f = f_;
    }

    ars430_ros_publisher::msg::RadarPacket out;
    out.header = in.header;
    out.event_id = in.event_id;
    out.time_stamp = in.time_stamp;
    out.measurement_counter = in.measurement_counter;
    out.vambig = in.vambig;

    const double snr_min = (in.event_id == EVENT_FAR) ? f.snr_min_far : f.snr_min_near;
    const double az_max = f.az_max_deg * M_PI / 180.0;

    for (const auto& d : in.detections) {
      stat_in_++;
      if (!f.raw) {
        if (d.snr < snr_min)                        { drop_snr_++;   continue; }
        const double v = std::fabs(d.vrel_rad);
        if (v < f.velocity_min || v > f.velocity_max) { drop_vel_++; continue; }
        if (d.range < f.range_min || d.range > f.range_max) { drop_rng_++; continue; }
        if (std::fabs(d.az_ang) > az_max)           { drop_az_++;    continue; }
        if (d.rcs < f.rcs_min)                      { drop_rcs_++;   continue; }
      }
      stat_out_++;
      out.detections.push_back(d);
    }

    if (!out.detections.empty()) {
      pub_->publish(out);
    }
  }

  void report() {
    if (stat_in_ == 0) return;
    RCLCPP_INFO(get_logger(),
                "last 5s: in=%lu kept=%lu (%.0f%%) | dropped snr=%lu vel=%lu range=%lu az=%lu rcs=%lu",
                (unsigned long)stat_in_, (unsigned long)stat_out_,
                100.0 * stat_out_ / stat_in_,
                (unsigned long)drop_snr_, (unsigned long)drop_vel_,
                (unsigned long)drop_rng_, (unsigned long)drop_az_,
                (unsigned long)drop_rcs_);
    stat_in_ = stat_out_ = drop_snr_ = drop_vel_ = drop_rng_ = drop_az_ = drop_rcs_ = 0;
  }

  Filters f_;
  std::mutex mtx_;
  uint64_t stat_in_ = 0, stat_out_ = 0;
  uint64_t drop_snr_ = 0, drop_vel_ = 0, drop_rng_ = 0, drop_az_ = 0, drop_rcs_ = 0;

  rclcpp::Publisher<ars430_ros_publisher::msg::RadarPacket>::SharedPtr pub_;
  rclcpp::Subscription<ars430_ros_publisher::msg::RadarPacket>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr report_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RadarProcessorNode>());
  rclcpp::shutdown();
  return 0;
}
