/* radar_visualizer (ROS2 Jazzy)
 *
 * Converts RadarPacket messages into sensor_msgs/PointCloud2 for RViz2.
 * Packets sharing the same radar time_stamp are grouped into one cloud.
 *
 * Cloud fields per point: x, y, z, intensity (RCS mapped to 0..100,
 * green=weak red=strong with the bundled RViz config) and velocity (m/s),
 * so RViz can color by movement to make walking people stand out.
 *
 * Parameters (startup): input_topic, output_topic.
 */

#include <cmath>
#include <cstring>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include "ars430_ros_publisher/msg/radar_packet.hpp"

static constexpr char BASE_FRAME[] = "radar_fixed";
static constexpr double FOV_NEAR_DEG = 70.0;
static constexpr double FOV_MAX_DIST = 25.0;

class RadarVisualizerNode : public rclcpp::Node {
public:
  RadarVisualizerNode() : Node("radar_visualizer") {
    const std::string in  = declare_parameter<std::string>("input_topic", "/filtered_radar_packet_1");
    const std::string out = declare_parameter<std::string>("output_topic", "/radar_pointcloud_1");

    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(out, rclcpp::QoS(10));
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", rclcpp::QoS(10));
    sub_ = create_subscription<ars430_ros_publisher::msg::RadarPacket>(
        in, rclcpp::QoS(50),
        [this](ars430_ros_publisher::msg::RadarPacket::ConstSharedPtr msg) { onPacket(msg); });

    // radar_fixed is rigidly attached to base_link
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now();
    t.header.frame_id = "base_link";
    t.child_frame_id = BASE_FRAME;
    t.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(t);

    RCLCPP_INFO(get_logger(), "Visualizing %s -> %s", in.c_str(), out.c_str());
  }

private:
  void onPacket(ars430_ros_publisher::msg::RadarPacket::ConstSharedPtr msg) {
    if (!group_.empty() && msg->time_stamp != group_ts_) {
      emitCloud();
    }
    group_ts_ = msg->time_stamp;
    group_.push_back(msg);
  }

  void emitCloud() {
    sensor_msgs::msg::PointCloud2 pc;
    pc.header.stamp = group_.front()->header.stamp;
    pc.header.frame_id = BASE_FRAME;
    pc.height = 1;
    pc.is_bigendian = false;
    pc.is_dense = true;
    pc.point_step = 20; // 5 x float32

    const char* names[5] = {"x", "y", "z", "intensity", "velocity"};
    for (int i = 0; i < 5; i++) {
      sensor_msgs::msg::PointField f;
      f.name = names[i];
      f.offset = 4 * i;
      f.datatype = sensor_msgs::msg::PointField::FLOAT32;
      f.count = 1;
      pc.fields.push_back(f);
    }

    size_t npts = 0;
    for (const auto& pkt : group_) npts += pkt->detections.size();
    pc.width = static_cast<uint32_t>(npts);
    pc.row_step = pc.point_step * pc.width;
    pc.data.resize(pc.row_step);

    uint8_t* dst = pc.data.data();
    for (const auto& pkt : group_) {
      for (const auto& d : pkt->detections) {
        // Map RCS from -100..+100 dBsm to 0..100 (same convention as ROS1 tooling)
        const float vals[5] = {d.pos_x, d.pos_y, d.pos_z,
                               d.rcs / 2.0f + 50.0f, d.vrel_rad};
        std::memcpy(dst, vals, sizeof(vals));
        dst += sizeof(vals);
      }
    }
    group_.clear();

    cloud_pub_->publish(pc);
    publishFovLines(pc.header.stamp);
  }

  void publishFovLines(const builtin_interfaces::msg::Time& stamp) {
    for (int side = 0; side < 2; side++) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = BASE_FRAME;
      m.header.stamp = stamp;
      m.ns = "radar_fov";
      m.id = side + 1;
      m.type = visualization_msgs::msg::Marker::LINE_STRIP;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.orientation.w = 1.0;
      m.scale.x = 0.1;
      m.color.b = 1.0;
      m.color.a = 1.0;
      geometry_msgs::msg::Point p0, p1;
      p1.x = FOV_MAX_DIST;
      p1.y = FOV_MAX_DIST * std::tan(FOV_NEAR_DEG * M_PI / 180.0) * (side ? 1.0 : -1.0);
      m.points.push_back(p0);
      m.points.push_back(p1);
      marker_pub_->publish(m);
    }
  }

  std::vector<ars430_ros_publisher::msg::RadarPacket::ConstSharedPtr> group_;
  uint32_t group_ts_ = 0;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Subscription<ars430_ros_publisher::msg::RadarPacket>::SharedPtr sub_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RadarVisualizerNode>());
  rclcpp::shutdown();
  return 0;
}
