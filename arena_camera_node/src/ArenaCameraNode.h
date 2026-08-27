#pragma once

// TODO
// - remove m_ before private members
// - add const to member functions
// fix includes in all files
// - should we rclcpp::shutdown in construction instead
//

// std
#include <chrono>      //chrono_literals
#include <functional>  // std::bind , std::placeholders
#include <mutex>       // std::mutex (runtime reconfigure)
#include <thread>      // acquisition thread (keeps executor free for params)
#include <vector>

// ros
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>           // WallTimer
#include <rcl_interfaces/msg/set_parameters_result.hpp>  // on_set_parameters
#include <sensor_msgs/msg/image.hpp>  //image msg published
#include <std_srvs/srv/trigger.hpp>   // Trigger

// arena sdk
#include "ArenaApi.h"

class ArenaCameraNode : public rclcpp::Node
{
 public:
  ArenaCameraNode() : Node("arena_camera_node")
  {
    // set stdout buffer size for ROS defined size BUFSIZE
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    log_info(std::string("Creating \"") + this->get_name() + "\" node");
    parse_parameters_();
    initialize_();
    log_info(std::string("Created \"") + this->get_name() + "\" node");
  }

  ~ArenaCameraNode()
  {
    // The acquisition loop runs on its own thread; make sure it has stopped
    // (rclcpp::ok() went false on shutdown) before the node is torn down.
    if (m_acquisition_thread_.joinable()) {
      m_acquisition_thread_.join();
    }
    log_info(std::string("Destroying \"") + this->get_name() + "\" node");
  }

  void log_debug(std::string msg) { RCLCPP_DEBUG(this->get_logger(), msg.c_str()); };
  void log_info(std::string msg) { RCLCPP_INFO(this->get_logger(), msg.c_str()); };
  void log_warn(std::string msg) { RCLCPP_WARN(this->get_logger(), msg.c_str()); };
  void log_err(std::string msg) { RCLCPP_ERROR(this->get_logger(), msg.c_str()); };

 private:
  std::shared_ptr<Arena::ISystem> m_pSystem;
  std::shared_ptr<Arena::IDevice> m_pDevice;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_pub_;
  rclcpp::TimerBase::SharedPtr m_wait_for_device_timer_callback_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_trigger_an_image_srv_;

  std::string serial_;
  bool is_passed_serial_;

  std::string topic_;

  std::string frame_id_;

  size_t width_;
  bool is_passed_width;

  size_t height_;
  bool is_passed_height;

  double gain_;
  bool is_passed_gain_;
  std::string gain_auto_;  // "Off" | "Once" | "Continuous" (case-insensitive)

  double exposure_time_;
  bool is_passed_exposure_time_;
  std::string exposure_auto_;  // "Off" | "Once" | "Continuous" (case-insensitive)

  // Caps the acquisition rate (AcquisitionFrameRate). Needed because the
  // camera otherwise free-runs as fast as the GigE link allows: switching
  // pixelformat from rgb8 to a Bayer format frees ~3x the bandwidth, and
  // without this cap the camera spends it on more frames instead of on the
  // headroom that keeps frame delivery regular. <= 0 means "don't cap".
  double frame_rate_;
  bool is_passed_frame_rate_;

  std::string pixelformat_pfnc_;
  std::string pixelformat_ros_;
  bool is_passed_pixelformat_ros_;

  bool trigger_mode_activated_;

  bool ptp_enable_;

  std::string pub_qos_history_;
  bool is_passed_pub_qos_history_;

  size_t pub_qos_history_depth_;
  bool is_passed_pub_qos_history_depth_;

  std::string pub_qos_reliability_;
  bool is_passed_pub_qos_reliability_;

  // --- runtime reconfigure (ros2 param set while streaming) ---------------
  // The acquisition loop (publish_images_) runs on this thread so that
  // rclcpp::spin() on the main thread stays free to service the parameter
  // set/get services. The parameter callback only flips dirty flags under the
  // mutex; the actual Arena SDK calls happen in the acquisition thread
  // (apply_pending_reconfig_), so the SDK is only ever touched from one thread.
  std::thread m_acquisition_thread_;
  std::mutex m_reconfig_mutex_;
  bool m_gain_value_dirty_ = false;
  bool m_gain_auto_dirty_ = false;
  bool m_exposure_value_dirty_ = false;
  bool m_exposure_auto_dirty_ = false;
  bool m_resolution_dirty_ = false;
  bool m_frame_rate_dirty_ = false;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      m_param_cb_handle_;

  void parse_parameters_();
  void initialize_();

  void wait_for_device_timer_callback_();

  void run_();
  // TODO :
  // - handle misconfigured device
  Arena::IDevice* create_device_ros_();
  void set_nodes_();
  void set_nodes_load_default_profile_();
  void set_nodes_ptp_();
  void set_nodes_roi_();
  void set_nodes_gain_();
  void set_nodes_pixelformat_();
  void set_nodes_exposure_();
  void set_nodes_frame_rate_();
  void set_nodes_trigger_mode_();
  void set_nodes_test_pattern_image_();
  void publish_images_();

  // runtime reconfigure
  rcl_interfaces::msg::SetParametersResult on_set_parameters_(
      const std::vector<rclcpp::Parameter>& params);
  void apply_pending_reconfig_();

  void publish_an_image_on_trigger_(
      std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void msg_form_image_(Arena::IImage* pImage,
                       sensor_msgs::msg::Image& image_msg);
};
