/* radar_clusters (ROS2 Jazzy)
 *
 * Builds labelled objects directly from the radar detections ("via B"):
 * gates detections by velocity, clusters them with DBSCAN, associates
 * clusters across cycles for persistence, and publishes one CUBE + label
 * per confirmed cluster on /radar/cluster_markers.
 *
 * Unlike the radar's internal tracker (radar_objects/service 230), these
 * cubes land exactly on the point cloud: same origin, same detections.
 * Designed for indoor person tracking.
 *
 * All thresholds are DYNAMIC parameters:
 *   velocity_min   m/s   only cluster moving detections (0 = cluster all)
 *   eps            m     DBSCAN neighbourhood radius
 *   min_points           DBSCAN minimum neighbours to form a cluster
 *   min_cycles           show a track only after N consecutive updates
 *   window_s       s     detection accumulation window (~2 radar cycles)
 *   assoc_dist     m     max centroid jump when re-associating a track
 */

#include <chrono>
#include <cmath>
#include <cstdio>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "ars430_ros_publisher/msg/radar_packet.hpp"

namespace {

struct RGB { float r, g, b; };

RGB colourForId(uint32_t id) {
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

struct Pt { float x, y, v; double t; };

struct Cluster {
  float cx = 0, cy = 0, vmean = 0, extent = 0;
  int n = 0;
};

// Plain O(n^2) DBSCAN; detection counts here are well under 1000
std::vector<Cluster> dbscan(const std::vector<Pt>& pts, float eps, int min_pts) {
  const int n = static_cast<int>(pts.size());
  std::vector<int> label(n, -1); // -1 = noise/unvisited
  const float eps2 = eps * eps;
  int next = 0;

  auto neighbours = [&](int i, std::vector<int>& out) {
    out.clear();
    for (int j = 0; j < n; j++) {
      const float dx = pts[i].x - pts[j].x, dy = pts[i].y - pts[j].y;
      if (dx * dx + dy * dy <= eps2) out.push_back(j);
    }
  };

  std::vector<int> nb, seed;
  for (int i = 0; i < n; i++) {
    if (label[i] != -1) continue;
    neighbours(i, nb);
    if (static_cast<int>(nb.size()) < min_pts) continue;
    const int cid = next++;
    label[i] = cid;
    seed = nb;
    for (size_t k = 0; k < seed.size(); k++) {
      const int j = seed[k];
      if (label[j] != -1) continue;
      label[j] = cid;
      neighbours(j, nb);
      if (static_cast<int>(nb.size()) >= min_pts) {
        seed.insert(seed.end(), nb.begin(), nb.end());
      }
    }
  }

  std::vector<Cluster> out(next);
  for (int i = 0; i < n; i++) {
    if (label[i] < 0) continue;
    Cluster& c = out[label[i]];
    c.cx += pts[i].x; c.cy += pts[i].y; c.vmean += std::fabs(pts[i].v); c.n++;
  }
  for (auto& c : out) {
    if (c.n) { c.cx /= c.n; c.cy /= c.n; c.vmean /= c.n; }
  }
  for (int i = 0; i < n; i++) {
    if (label[i] < 0) continue;
    Cluster& c = out[label[i]];
    const float d = std::hypot(pts[i].x - c.cx, pts[i].y - c.cy);
    if (d > c.extent) c.extent = d;
  }
  return out;
}

struct Track {
  uint32_t id;
  float cx, cy, vmean;
  uint32_t age = 1;
  uint32_t missed = 0;
};

}  // namespace

class RadarClustersNode : public rclcpp::Node {
public:
  RadarClustersNode() : Node("radar_clusters") {
    const int id = static_cast<int>(declare_parameter<int64_t>("id", 1));
    input_topic_ = declare_parameter<std::string>("input_topic",
                       "/filtered_radar_packet_" + std::to_string(id));

    velocity_min_      = declare_parameter<double>("velocity_min", 0.3);
    eps_               = declare_parameter<double>("eps", 1.0);
    min_points_        = declare_parameter<int64_t>("min_points", 4);
    min_cycles_        = declare_parameter<int64_t>("min_cycles", 2);
    window_s_          = declare_parameter<double>("window_s", 0.2);
    assoc_dist_        = declare_parameter<double>("assoc_dist", 1.0);
    marker_size_z_     = declare_parameter<double>("marker_size_z", 1.7);
    marker_alpha_      = declare_parameter<double>("marker_alpha", 0.7);
    marker_lifetime_s_ = declare_parameter<double>("marker_lifetime_s", 0.4);
    show_labels_       = declare_parameter<bool>("show_labels", true);

    param_cb_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params) {
          rcl_interfaces::msg::SetParametersResult res;
          res.successful = true;
          std::lock_guard<std::mutex> lk(mtx_);
          for (const auto& p : params) {
            const auto& n = p.get_name();
            if      (n == "velocity_min")      velocity_min_ = p.as_double();
            else if (n == "eps")               eps_ = p.as_double();
            else if (n == "min_points")        min_points_ = p.as_int();
            else if (n == "min_cycles")        min_cycles_ = p.as_int();
            else if (n == "window_s")          window_s_ = p.as_double();
            else if (n == "assoc_dist")        assoc_dist_ = p.as_double();
            else if (n == "marker_size_z")     marker_size_z_ = p.as_double();
            else if (n == "marker_alpha")      marker_alpha_ = p.as_double();
            else if (n == "marker_lifetime_s") marker_lifetime_s_ = p.as_double();
            else if (n == "show_labels")       show_labels_ = p.as_bool();
            else if (n == "id" || n == "input_topic") {
              res.successful = false;
              res.reason = n + " is fixed at startup";
            }
          }
          return res;
        });

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/radar/cluster_markers", rclcpp::QoS(10));
    sub_ = create_subscription<ars430_ros_publisher::msg::RadarPacket>(
        input_topic_, rclcpp::QoS(50),
        [this](ars430_ros_publisher::msg::RadarPacket::ConstSharedPtr msg) { onPacket(*msg); });

    cluster_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] { runClustering(); });
    report_timer_ = create_wall_timer(std::chrono::seconds(5), [this] { report(); });
    RCLCPP_INFO(get_logger(), "Clustering %s (velocity_min=%.2f eps=%.2f min_points=%ld)",
                input_topic_.c_str(), velocity_min_, eps_, (long)min_points_);
  }

private:
  void onPacket(const ars430_ros_publisher::msg::RadarPacket& msg) {
    double vmin;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      vmin = velocity_min_;
    }
    const double t = now().seconds();
    std::lock_guard<std::mutex> lk(win_mtx_);
    for (const auto& d : msg.detections) {
      if (std::fabs(d.vrel_rad) < vmin) continue;
      window_.push_back({d.pos_x, d.pos_y, d.vrel_rad, t});
    }
    last_frame_ = msg.header.frame_id;
  }

  void runClustering() {
    double eps, window_s, assoc, size_z, alpha, lifetime;
    int64_t min_pts, min_cycles;
    bool show_labels;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      eps = eps_; min_pts = min_points_; min_cycles = min_cycles_;
      window_s = window_s_; assoc = assoc_dist_;
      size_z = marker_size_z_; alpha = marker_alpha_;
      lifetime = marker_lifetime_s_; show_labels = show_labels_;
    }

    std::vector<Pt> pts;
    {
      std::lock_guard<std::mutex> lk(win_mtx_);
      const double cutoff = now().seconds() - window_s;
      while (!window_.empty() && window_.front().t < cutoff) window_.pop_front();
      pts.assign(window_.begin(), window_.end());
    }
    stat_points_ += pts.size();

    const auto clusters = dbscan(pts, static_cast<float>(eps), static_cast<int>(min_pts));

    // Associate clusters to existing tracks by nearest centroid
    std::vector<bool> used(tracks_.size(), false);
    std::vector<Track> updated;
    for (const auto& c : clusters) {
      if (c.n == 0) continue;
      stat_clusters_++;
      int best = -1;
      float bestd = static_cast<float>(assoc);
      for (size_t i = 0; i < tracks_.size(); i++) {
        if (used[i]) continue;
        const float d = std::hypot(c.cx - tracks_[i].cx, c.cy - tracks_[i].cy);
        if (d < bestd) { bestd = d; best = static_cast<int>(i); }
      }
      Track tr{};
      if (best >= 0) {
        used[best] = true;
        tr = tracks_[best];
        tr.age++;
        tr.missed = 0;
      } else {
        tr.id = next_id_++;
        tr.age = 1;
      }
      tr.cx = c.cx; tr.cy = c.cy; tr.vmean = c.vmean;
      updated.push_back(tr);

      if (tr.age >= static_cast<uint32_t>(min_cycles)) {
        publishTrack(tr, c, size_z, alpha, lifetime, show_labels);
      }
    }
    // Keep unmatched tracks briefly so a blink doesn't reset persistence
    for (size_t i = 0; i < tracks_.size(); i++) {
      if (!used[i] && tracks_[i].missed < 3) {
        Track tr = tracks_[i];
        tr.missed++;
        updated.push_back(tr);
      }
    }
    tracks_ = std::move(updated);
  }

  void publishTrack(const Track& tr, const Cluster& c,
                    double size_z, double alpha, double lifetime, bool show_labels) {
    stat_shown_++;
    visualization_msgs::msg::MarkerArray arr;
    const RGB col = colourForId(tr.id);
    const double size_xy = std::min(std::max(2.0 * c.extent, 0.4), 1.5);

    visualization_msgs::msg::Marker cube;
    cube.header.stamp = now();
    cube.header.frame_id = last_frame_.empty() ? "radar_fixed" : last_frame_;
    cube.ns = "radar_clusters";
    cube.id = static_cast<int>(tr.id) * 2;
    cube.type = visualization_msgs::msg::Marker::CUBE;
    cube.action = visualization_msgs::msg::Marker::ADD;
    cube.pose.position.x = tr.cx;
    cube.pose.position.y = tr.cy;
    cube.pose.position.z = size_z / 2.0;
    cube.pose.orientation.w = 1.0;
    cube.scale.x = size_xy;
    cube.scale.y = size_xy;
    cube.scale.z = size_z;
    cube.color.r = col.r; cube.color.g = col.g; cube.color.b = col.b;
    cube.color.a = static_cast<float>(alpha);
    cube.lifetime = rclcpp::Duration::from_seconds(lifetime);
    arr.markers.push_back(cube);

    if (show_labels) {
      visualization_msgs::msg::Marker text = cube;
      text.ns = "radar_clusters_labels";
      text.id = static_cast<int>(tr.id) * 2 + 1;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.pose.position.z = size_z + 0.35;
      text.scale.x = text.scale.y = 0.0;
      text.scale.z = 0.4;
      text.color.r = text.color.g = text.color.b = 1.0f;
      text.color.a = 1.0f;
      char buf[48];
      snprintf(buf, sizeof(buf), "P%u  %.1f m/s (%d pts)", tr.id, tr.vmean, c.n);
      text.text = buf;
      arr.markers.push_back(text);
    }
    marker_pub_->publish(arr);
  }

  void report() {
    RCLCPP_INFO(get_logger(),
                "last 5s: gated points=%lu | clusters=%lu | cubes shown=%lu | live tracks=%zu",
                (unsigned long)stat_points_, (unsigned long)stat_clusters_,
                (unsigned long)stat_shown_, tracks_.size());
    stat_points_ = stat_clusters_ = stat_shown_ = 0;
  }

  std::string input_topic_, last_frame_;
  double velocity_min_, eps_, window_s_, assoc_dist_;
  double marker_size_z_, marker_alpha_, marker_lifetime_s_;
  int64_t min_points_, min_cycles_;
  bool show_labels_ = true;

  std::mutex mtx_, win_mtx_;
  std::deque<Pt> window_;
  std::vector<Track> tracks_;
  uint32_t next_id_ = 1;
  uint64_t stat_points_ = 0, stat_clusters_ = 0, stat_shown_ = 0;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<ars430_ros_publisher::msg::RadarPacket>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr cluster_timer_, report_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RadarClustersNode>());
  rclcpp::shutdown();
  return 0;
}
