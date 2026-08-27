#include <algorithm>  // std::transform
#include <cctype>     // std::tolower
#include <cstring>    // memcopy
#include <stdexcept>  // std::runtime_err
#include <string>

// ROS
#include "rmw/types.h"

// ArenaSDK
#include "ArenaCameraNode.h"
#include "light_arena/deviceinfo_helper.h"
#include "rclcpp_adapter/pixelformat_translation.h"
#include "rclcpp_adapter/quilty_of_service_translation.cpp"

namespace {
// Case-insensitive helper so gain_auto/exposure_auto accept "off"/"Off"/"OFF"
// etc. Returns the lowercased copy.
std::string to_lower_(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  return s;
}
}  // namespace

void ArenaCameraNode::parse_parameters_()
{
  std::string nextParameterToDeclare = "";
  try {
    nextParameterToDeclare = "serial";
    serial_ = this->declare_parameter<std::string>("serial", "");
    is_passed_serial_ = serial_ != "";

    nextParameterToDeclare = "pixelformat";
    pixelformat_ros_ = this->declare_parameter("pixelformat", "");
    is_passed_pixelformat_ros_ = pixelformat_ros_ != "";

    nextParameterToDeclare = "width";
    width_ = this->declare_parameter("width", 0);
    is_passed_width = width_ > 0;

    nextParameterToDeclare = "height";
    height_ = this->declare_parameter("height", 0);
    is_passed_height = height_ > 0;

    nextParameterToDeclare = "gain";
    gain_ = this->declare_parameter("gain", -1.0);
    is_passed_gain_ = gain_ >= 0;

    // "Off" -> use the fixed `gain` value; "Continuous"/"Once" -> auto gain.
    // Runtime-modifiable: `ros2 param set <node> gain_auto Continuous` or
    // `ros2 param set <node> gain 13.0` (setting a value flips it to manual).
    nextParameterToDeclare = "gain_auto";
    gain_auto_ = this->declare_parameter<std::string>("gain_auto", "Continuous");

    nextParameterToDeclare = "exposure_time";
    exposure_time_ = this->declare_parameter("exposure_time", -1.0);
    is_passed_exposure_time_ = exposure_time_ >= 0;

    // Same idea as gain_auto, for exposure. "Off" -> fixed `exposure_time`.
    nextParameterToDeclare = "exposure_auto";
    exposure_auto_ =
        this->declare_parameter<std::string>("exposure_auto", "Continuous");

    // Cap on AcquisitionFrameRate (fps). <= 0 leaves the camera free-running.
    // Runtime-modifiable: `ros2 param set <node> frame_rate 12.0`.
    nextParameterToDeclare = "frame_rate";
    frame_rate_ = this->declare_parameter("frame_rate", -1.0);
    is_passed_frame_rate_ = frame_rate_ > 0;

    nextParameterToDeclare = "trigger_mode";
    trigger_mode_activated_ = this->declare_parameter("trigger_mode", false);
    // no need to is_passed_trigger_mode_ because it is already a boolean

    nextParameterToDeclare = "ptp_enable";
    ptp_enable_ = this->declare_parameter("ptp_enable", true);
    // no need for is_passed_ptp_enable_ because it is already a boolean

    nextParameterToDeclare = "topic";
    topic_ = this->declare_parameter(
        "topic", std::string("/") + this->get_name() + "/images");
    // no need to is_passed_topic_

    nextParameterToDeclare = "frame_id";
    // Defaults to the node name (e.g. "arena_cam_enp4s0") rather than
    // anything camera-specific -- must be set explicitly per camera in the
    // launch file to match the static_transform_publisher tree (see
    // eod_av_launch/bringup.py), same as the other
    // 2 Tritons and the Hesai.
    frame_id_ = this->declare_parameter("frame_id", this->get_name());

    nextParameterToDeclare = "qos_history";
    pub_qos_history_ = this->declare_parameter("qos_history", "");
    is_passed_pub_qos_history_ = pub_qos_history_ != "";

    nextParameterToDeclare = "qos_history_depth";
    pub_qos_history_depth_ = this->declare_parameter("qos_history_depth", 0);
    is_passed_pub_qos_history_depth_ = pub_qos_history_depth_ > 0;

    nextParameterToDeclare = "qos_reliability";
    pub_qos_reliability_ = this->declare_parameter("qos_reliability", "");
    is_passed_pub_qos_reliability_ = pub_qos_reliability_ != "";

  } catch (rclcpp::ParameterTypeException& e) {
    log_err(nextParameterToDeclare + " argument");
    throw;
  }
}

void ArenaCameraNode::initialize_()
{
  using namespace std::chrono_literals;
  // ARENASDK ---------------------------------------------------------------
  // Custom deleter for system
  m_pSystem =
      std::shared_ptr<Arena::ISystem>(nullptr, [=](Arena::ISystem* pSystem) {
        if (pSystem) {  // this is an issue for multi devices
          Arena::CloseSystem(pSystem);
          log_info("System is destroyed");
        }
      });
  m_pSystem.reset(Arena::OpenSystem());

  // Custom deleter for device
  m_pDevice =
      std::shared_ptr<Arena::IDevice>(nullptr, [=](Arena::IDevice* pDevice) {
        if (m_pSystem && pDevice) {
          m_pSystem->DestroyDevice(pDevice);
          log_info("Device is destroyed");
        }
      });

  //
  // CHECK DEVICE CONNECTION ( timer ) --------------------------------------
  //
  // TODO
  // - Think of design that allow the node to start stream as soon as
  // it is initialized without waiting for spin to be called
  // - maybe change 1s to a smaller value
  m_wait_for_device_timer_callback_ = this->create_wall_timer(
      1s, std::bind(&ArenaCameraNode::wait_for_device_timer_callback_, this));

  //
  // TRIGGER (service) ------------------------------------------------------
  //
  using namespace std::placeholders;
  m_trigger_an_image_srv_ = this->create_service<std_srvs::srv::Trigger>(
      std::string(this->get_name()) + "/trigger_image",
      std::bind(&ArenaCameraNode::publish_an_image_on_trigger_, this, _1, _2));

  //
  // RUNTIME RECONFIGURE (ros2 param set) ----------------------------------
  //
  // The callback only records what changed (dirty flags); the acquisition
  // thread applies it to the camera between frames (apply_pending_reconfig_).
  m_param_cb_handle_ = this->add_on_set_parameters_callback(
      std::bind(&ArenaCameraNode::on_set_parameters_, this, _1));

  //
  // Publisher --------------------------------------------------------------
  //
  // m_pub_qos is rclcpp::SensorDataQoS has these defaults
  // https://github.com/ros2/rmw/blob/fb06b57975373b5a23691bb00eb39c07f1660ed7/rmw/include/rmw/qos_profiles.h#L25

  /*
  static const rmw_qos_profile_t rmw_qos_profile_sensor_data =
  {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    5, // history depth
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false // avoid ros namespace conventions
  };
  */
  rclcpp::SensorDataQoS pub_qos_;
  // QoS history
  if (is_passed_pub_qos_history_) {
    if (is_supported_qos_histroy_policy(pub_qos_history_)) {
      pub_qos_.history(
          K_CMDLN_PARAMETER_TO_QOS_HISTORY_POLICY[pub_qos_history_]);
    } else {
      log_err(pub_qos_history_ + " is not supported for this node");
      // A bare `throw;` here has no in-flight exception to re-raise, so it
      // would call std::terminate(). Throw a real exception instead so the
      // bad qos_history value surfaces as a readable error.
      throw std::invalid_argument(
          "unsupported qos_history: " + pub_qos_history_);
    }
  }
  // QoS depth
  if (is_passed_pub_qos_history_depth_ &&
      K_CMDLN_PARAMETER_TO_QOS_HISTORY_POLICY[pub_qos_history_] ==
          RMW_QOS_POLICY_HISTORY_KEEP_LAST) {
    // TODO
    // test err msg withwhen -1
    pub_qos_.keep_last(pub_qos_history_depth_);
  }

  // Qos reliability
  if (is_passed_pub_qos_reliability_) {
    if (is_supported_qos_reliability_policy(pub_qos_reliability_)) {
      pub_qos_.reliability(
          K_CMDLN_PARAMETER_TO_QOS_RELIABILITY_POLICY[pub_qos_reliability_]);
    } else {
      log_err(pub_qos_reliability_ + " is not supported for this node");
      // Same as above: avoid a bare `throw;` with no active exception.
      throw std::invalid_argument(
          "unsupported qos_reliability: " + pub_qos_reliability_);
    }
  }

  // rmw_qos_history_policy_t history_policy_ = RMW_QOS_
  // rmw_qos_history_policy_t;
  // auto pub_qos_init = rclcpp::QoSInitialization(history_policy_, );

  m_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      this->get_parameter("topic").as_string(), pub_qos_);

  std::stringstream pub_qos_info;
  auto pub_qos_profile = pub_qos_.get_rmw_qos_profile();
  pub_qos_info
      << '\t' << "QoS history     = "
      << K_QOS_HISTORY_POLICY_TO_CMDLN_PARAMETER[pub_qos_profile.history]
      << '\n';
  pub_qos_info << "\t\t\t\t"
               << "QoS depth       = " << pub_qos_profile.depth << '\n';
  pub_qos_info << "\t\t\t\t"
               << "QoS reliability = "
               << K_QOS_RELIABILITY_POLICY_TO_CMDLN_PARAMETER[pub_qos_profile
                                                                  .reliability]
               << '\n';

  log_info(pub_qos_info.str());
}

void ArenaCameraNode::wait_for_device_timer_callback_()
{
  // something happend while checking for cameras
  if (!rclcpp::ok()) {
    log_err("Interrupted while waiting for arena camera. Exiting.");
    rclcpp::shutdown();
  }

  // camera discovery
  m_pSystem->UpdateDevices(100);  // in millisec
  auto device_infos = m_pSystem->GetDevices();

  // no camera is connected
  if (!device_infos.size()) {
    log_info("No arena camera is connected. Waiting for device(s)...");
  }
  // at least on is found
  else {
    m_wait_for_device_timer_callback_->cancel();
    log_info(std::to_string(device_infos.size()) +
             " arena device(s) has been discoved.");
    run_();
  }
}

void ArenaCameraNode::run_()
{
  auto device = create_device_ros_();
  m_pDevice.reset(device);
  set_nodes_();
  m_pDevice->StartStream();

  if (!trigger_mode_activated_) {
    // Run acquisition on its own thread so rclcpp::spin() on the main thread
    // stays responsive to parameter services -- otherwise the blocking
    // publish loop would starve `ros2 param set` (gain/exposure/resolution).
    m_acquisition_thread_ =
        std::thread(&ArenaCameraNode::publish_images_, this);
  } else {
    // else ros::spin will
  }
}

void ArenaCameraNode::publish_images_()
{
  Arena::IImage* pImage = nullptr;
  while (rclcpp::ok()) {
    try {
      // Apply any pending `ros2 param set` (gain/exposure live; resolution
      // stops+restarts the stream) between frames, on this same thread so the
      // Arena SDK is only ever touched here.
      apply_pending_reconfig_();

      auto p_image_msg = std::make_unique<sensor_msgs::msg::Image>();
      pImage = m_pDevice->GetImage(1000);
      msg_form_image_(pImage, *p_image_msg);

      m_pub_->publish(std::move(p_image_msg));

      log_info(std::string("image ") + std::to_string(pImage->GetFrameId()) +
               " published to " + topic_);
      this->m_pDevice->RequeueBuffer(pImage);

    } catch (std::exception& e) {
      if (pImage) {
        this->m_pDevice->RequeueBuffer(pImage);
        pImage = nullptr;
        log_warn(std::string("Exception occurred while publishing an image\n") +
                 e.what());
      }
    }
  };
}

void ArenaCameraNode::msg_form_image_(Arena::IImage* pImage,
                                      sensor_msgs::msg::Image& image_msg)
{
  try {
    // 1 ) Header
    //      - stamp.sec
    //      - stamp.nanosec
    //      - Frame ID
    image_msg.header.stamp.sec =
        static_cast<uint32_t>(pImage->GetTimestampNs() / 1000000000);
    image_msg.header.stamp.nanosec =
        static_cast<uint32_t>(pImage->GetTimestampNs() % 1000000000);
    image_msg.header.frame_id = frame_id_;

    //
    // 2 ) Height
    //
    image_msg.height = height_;

    //
    // 3 ) Width
    //
    image_msg.width = width_;

    //
    // 4 ) encoding
    //
    image_msg.encoding = pixelformat_ros_;

    //
    // 5 ) is_big_endian
    //
    // TODO what to do if unknown
    image_msg.is_bigendian = pImage->GetPixelEndianness() ==
                             Arena::EPixelEndianness::PixelEndiannessBig;
    //
    // 6 ) step
    //
    // TODO could be optimized by moving it out
    auto pixel_length_in_bytes = pImage->GetBitsPerPixel() / 8;
    auto width_length_in_bytes = pImage->GetWidth() * pixel_length_in_bytes;
    image_msg.step =
        static_cast<sensor_msgs::msg::Image::_step_type>(width_length_in_bytes);

    //
    // 7) data
    //
    auto image_data_length_in_bytes = width_length_in_bytes * height_;
    image_msg.data.resize(image_data_length_in_bytes);
    auto x = pImage->GetData();
    std::memcpy(&image_msg.data[0], pImage->GetData(),
                image_data_length_in_bytes);

  } catch (...) {
    log_warn(
        "Failed to create Image ROS MSG. Published Image Msg might be "
        "corrupted");
  }
}

void ArenaCameraNode::publish_an_image_on_trigger_(
    std::shared_ptr<std_srvs::srv::Trigger::Request> request /*unused*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!trigger_mode_activated_) {
    std::string msg =
        "Failed to trigger image because the device is not in trigger mode."
        "run `ros2 run arena_camera_node run --ros-args -p trigger_mode:=true`";
    log_warn(msg);
    response->message = msg;
    response->success = false;
    // Not in trigger mode: report failure and stop. Without this return the
    // code fell through and tried to trigger anyway.
    return;
  }

  log_info("A client triggered an image request");

  Arena::IImage* pImage = nullptr;
  try {
    // trigger
    bool triggerArmed = false;
    auto waitForTriggerCount = 10;
    do {
      // infinite loop when I step in (sometimes)
      triggerArmed =
          Arena::GetNodeValue<bool>(m_pDevice->GetNodeMap(), "TriggerArmed");

      if (triggerArmed == false && (waitForTriggerCount % 10) == 0) {
        log_info("waiting for trigger to be armed");
      }

    } while (triggerArmed == false);

    log_debug("trigger is armed; triggering an image");
    Arena::ExecuteNode(m_pDevice->GetNodeMap(), "TriggerSoftware");

    // get image
    auto p_image_msg = std::make_unique<sensor_msgs::msg::Image>();

    log_debug("getting an image");
    pImage = m_pDevice->GetImage(1000);
    auto msg = std::string("image ") + std::to_string(pImage->GetFrameId()) +
               " published to " + topic_;
    msg_form_image_(pImage, *p_image_msg);
    m_pub_->publish(std::move(p_image_msg));
    response->message = msg;
    response->success = true;

    log_info(msg);
    this->m_pDevice->RequeueBuffer(pImage);

  }

  catch (std::exception& e) {
    if (pImage) {
      this->m_pDevice->RequeueBuffer(pImage);
      pImage = nullptr;
    }
    auto msg =
        std::string("Exception occurred while grabbing an image\n") + e.what();
    log_warn(msg);
    response->message = msg;
    response->success = false;

  }

  catch (GenICam::GenericException& e) {
    if (pImage) {
      this->m_pDevice->RequeueBuffer(pImage);
      pImage = nullptr;
    }
    auto msg =
        std::string("GenICam Exception occurred while grabbing an image\n") +
        e.what();
    log_warn(msg);
    response->message = msg;
    response->success = false;
  }
}

Arena::IDevice* ArenaCameraNode::create_device_ros_()
{
  m_pSystem->UpdateDevices(100);  // in millisec
  auto device_infos = m_pSystem->GetDevices();
  if (!device_infos.size()) {
    // TODO: handel disconnection
    throw std::runtime_error(
        "camera(s) were disconnected after they were discovered");
  }

  auto index = 0;
  if (is_passed_serial_) {
    index = DeviceInfoHelper::get_index_of_serial(device_infos, serial_);
  }

  auto pDevice = m_pSystem->CreateDevice(device_infos.at(index));
  log_info(std::string("device created ") +
           DeviceInfoHelper::info(device_infos.at(index)));
  return pDevice;
}

void ArenaCameraNode::set_nodes_()
{
  set_nodes_load_default_profile_();
  set_nodes_ptp_();
  set_nodes_roi_();
  set_nodes_gain_();
  set_nodes_pixelformat_();
  set_nodes_exposure_();
  // After exposure: the max achievable AcquisitionFrameRate depends on the
  // exposure time, so the cap has to be applied once exposure is settled.
  set_nodes_frame_rate_();
  set_nodes_trigger_mode_();
  //set_nodes_test_pattern_image_();
}

void ArenaCameraNode::set_nodes_load_default_profile_()
{
  auto nodemap = m_pDevice->GetNodeMap();
  // device run on default profile all the time if no args are passed
  // otherwise, overwise only these params
  Arena::SetNodeValue<GenICam::gcstring>(nodemap, "UserSetSelector", "Default");
  // execute the profile
  Arena::ExecuteNode(nodemap, "UserSetLoad");
  log_info("\tdefault profile is loaded");
}

void ArenaCameraNode::set_nodes_ptp_()
{
  auto nodemap = m_pDevice->GetNodeMap();

  if (!ptp_enable_) {
    log_info("\tPTP disabled (ptp_enable:=false)");
    return;
  }

  try {
    Arena::SetNodeValue<bool>(nodemap, "PtpEnable", true);
    log_info("\tPtpEnable set to true");
  } catch (GenICam::GenericException& e) {
    log_err(std::string("\tFailed to set PtpEnable, continuing without PTP: ") +
            e.what());
    return;
  }

  // PtpSlaveOnly isn't in the confirmed-stable SDK API surface for every
  // camera model/firmware — set defensively so a missing node never blocks
  // startup.
  try {
    Arena::SetNodeValue<bool>(nodemap, "PtpSlaveOnly", true);
    log_info("\tPtpSlaveOnly set to true");
  } catch (GenICam::GenericException& e) {
    log_warn(std::string("\tPtpSlaveOnly not available, skipping: ") +
             e.what());
  }

  try {
    GenICam::gcstring status =
        Arena::GetNodeValue<GenICam::gcstring>(nodemap, "PtpStatus");
    log_info(std::string("\tPtpStatus = ") + std::string(status.c_str()));
  } catch (GenICam::GenericException& e) {
    log_warn(std::string("\tFailed to read PtpStatus: ") + e.what());
  }
}

void ArenaCameraNode::set_nodes_roi_()
{
  auto nodemap = m_pDevice->GetNodeMap();

  // Width -------------------------------------------------
  if (is_passed_width) {
    Arena::SetNodeValue<int64_t>(nodemap, "Width", width_);
  } else {
    width_ = Arena::GetNodeValue<int64_t>(nodemap, "Width");
  }

  // Height ------------------------------------------------
  if (is_passed_height) {
    Arena::SetNodeValue<int64_t>(nodemap, "Height", height_);
  } else {
    height_ = Arena::GetNodeValue<int64_t>(nodemap, "Height");
  }

  // TODO only if it was passed by ros arg
  log_info(std::string("\tROI set to ") + std::to_string(width_) + "X" +
           std::to_string(height_));
}

void ArenaCameraNode::set_nodes_gain_()
{
    auto nodemap = m_pDevice->GetNodeMap();

    // gain_auto is the master switch (runtime-modifiable via ros2 param set):
    //   "Off"        -> fixed gain, taken from the `gain` parameter
    //   "Continuous" -> camera auto-gain, tracking continuously (default)
    //   "Once"       -> auto-gain once, then hold
    const std::string mode = to_lower_(gain_auto_);
    if (mode == "off" || mode == "false") {
      Arena::SetNodeValue<GenICam::gcstring>(nodemap, "GainAuto", "Off");
      if (is_passed_gain_) {
        Arena::SetNodeValue<double>(nodemap, "Gain", gain_);
        log_info(std::string("\tGainAuto Off, Gain fixed at ") +
                 std::to_string(gain_));
      } else {
        log_info("\tGainAuto Off (no gain value passed; camera keeps current)");
      }
    } else if (mode == "once") {
      Arena::SetNodeValue<GenICam::gcstring>(nodemap, "GainAuto", "Once");
      log_info("\tGainAuto set to Once");
    } else {
      Arena::SetNodeValue<GenICam::gcstring>(nodemap, "GainAuto", "Continuous");
      log_info("\tGainAuto set to Continuous");
    }
}

void ArenaCameraNode::set_nodes_pixelformat_()
{
  auto nodemap = m_pDevice->GetNodeMap();
  // TODO ---------------------------------------------------------------------
  // PIXEL FORMAT HANDLEING

  if (is_passed_pixelformat_ros_) {
    pixelformat_pfnc_ = K_ROS2_PIXELFORMAT_TO_PFNC[pixelformat_ros_];
    if (pixelformat_pfnc_.empty()) {
      throw std::invalid_argument("pixelformat is not supported!");
    }

    try {
      Arena::SetNodeValue<GenICam::gcstring>(nodemap, "PixelFormat",
                                             pixelformat_pfnc_.c_str());
      log_info(std::string("\tPixelFormat set to ") + pixelformat_pfnc_);

    } catch (GenICam::GenericException& e) {
      // TODO
      // an rcl expectation might be expected
      auto x = std::string("pixelformat is not supported by this camera");
      x.append(e.what());
      throw std::invalid_argument(x);
    }
  } else {
    pixelformat_pfnc_ =
        Arena::GetNodeValue<GenICam::gcstring>(nodemap, "PixelFormat");
    pixelformat_ros_ = K_PFNC_TO_ROS2_PIXELFORMAT[pixelformat_pfnc_];

    if (pixelformat_ros_.empty()) {
      log_warn(
          "the device current pixelfromat value is not supported by ROS2. "
          "please use --ros-args -p pixelformat:=\"<supported pixelformat>\".");
      // TODO
      // print list of supported pixelformats
    }
  }
}

void ArenaCameraNode::set_nodes_exposure_()
{
  auto nodemap = m_pDevice->GetNodeMap();

  // exposure_auto mirrors gain_auto (runtime-modifiable):
  //   "Off"        -> fixed exposure from the `exposure_time` parameter (us)
  //   "Continuous" -> camera auto-exposure, tracking continuously (default)
  //   "Once"       -> auto-exposure once, then hold
  const std::string mode = to_lower_(exposure_auto_);
  if (mode == "off" || mode == "false") {
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "ExposureAuto", "Off");
    if (is_passed_exposure_time_) {
      Arena::SetNodeValue<double>(nodemap, "ExposureTime", exposure_time_);
      log_info(std::string("\tExposureAuto Off, ExposureTime fixed at ") +
               std::to_string(exposure_time_));
    } else {
      log_info(
          "\tExposureAuto Off (no exposure_time passed; camera keeps current)");
    }
  } else if (mode == "once") {
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "ExposureAuto", "Once");
    log_info("\tExposureAuto set to Once");
  } else {
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "ExposureAuto", "Continuous");
    log_info("\tExposureAuto set to Continuous");
  }
}

void ArenaCameraNode::set_nodes_frame_rate_()
{
  auto nodemap = m_pDevice->GetNodeMap();

  if (!is_passed_frame_rate_) {
    // Free-running: the camera streams as fast as exposure + link allow. With
    // a Bayer pixelformat that can be ~3x the rgb8 rate, which eats the very
    // bandwidth headroom that keeps frame intervals regular. Prefer capping.
    try {
      Arena::SetNodeValue<bool>(nodemap, "AcquisitionFrameRateEnable", false);
    } catch (GenICam::GenericException& e) {
      // Not every model exposes the enable node; harmless.
    }
    log_info("\tframe_rate not capped (camera free-runs)");
    return;
  }

  try {
    Arena::SetNodeValue<bool>(nodemap, "AcquisitionFrameRateEnable", true);
    Arena::SetNodeValue<double>(nodemap, "AcquisitionFrameRate", frame_rate_);
    log_info(std::string("\tAcquisitionFrameRate capped at ") +
             std::to_string(frame_rate_) + " fps");
  } catch (GenICam::GenericException& e) {
    // Asking for more than the current exposure/link allows throws here
    // instead of clamping; keep streaming rather than aborting startup.
    log_warn(std::string("\tCould not set AcquisitionFrameRate to ") +
             std::to_string(frame_rate_) +
             " fps (probably above what the current exposure/link allows): " +
             e.what());
  }
}

void ArenaCameraNode::set_nodes_trigger_mode_()
{
  auto nodemap = m_pDevice->GetNodeMap();
  if (trigger_mode_activated_) {
    if (exposure_time_ < 0) {
      log_warn(
          "\tavoid long waits wating for triggered images by providing proper "
          "exposure_time.");
    }
    // Enable trigger mode before setting the source and selector
    // and before starting the stream. Trigger mode cannot be turned
    // on and off while the device is streaming.

    // Make sure Trigger Mode set to 'Off' after finishing this example
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerMode", "On");

    // Set the trigger source to software in order to trigger buffers
    // without the use of any additional hardware.
    // Lines of the GPIO can also be used to trigger.
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerSource",
                                           "Software");
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerSelector",
                                           "FrameStart");
    auto msg =
        std::string(
            "\ttrigger_mode is activated. To trigger an image run `ros2 run ") +
        this->get_name() + " trigger_image`";
    log_warn(msg);
  }
  // unset device from being in trigger mode if user did not pass trigger
  // mode parameter because the trigger nodes are not rest when loading
  // the user default profile
  else {
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerMode", "Off");
  }
}

rcl_interfaces::msg::SetParametersResult ArenaCameraNode::on_set_parameters_(
    const std::vector<rclcpp::Parameter>& params)
{
  // Runs on the executor (main) thread. Validate and record WHAT changed; the
  // acquisition thread applies it to the camera between frames. Do NOT touch
  // the Arena SDK here -- only the acquisition thread may.
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  bool gain_v = false, gain_a = false, exp_v = false, exp_a = false, res = false,
       fps = false;
  for (const auto& p : params) {
    const std::string& name = p.get_name();
    if (name == "gain") {
      if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        result.successful = false;
        result.reason = "gain must be a double (e.g. 13.0)";
        break;
      }
      gain_v = true;
    } else if (name == "gain_auto") {
      gain_a = true;
    } else if (name == "exposure_time") {
      if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        result.successful = false;
        result.reason = "exposure_time must be a double (microseconds)";
        break;
      }
      exp_v = true;
    } else if (name == "exposure_auto") {
      exp_a = true;
    } else if (name == "frame_rate") {
      if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        result.successful = false;
        result.reason = "frame_rate must be a double (fps, <=0 to uncap)";
        break;
      }
      fps = true;
    } else if (name == "width" || name == "height") {
      if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER ||
          p.as_int() <= 0) {
        result.successful = false;
        result.reason = name + " must be a positive integer";
        break;
      }
      res = true;
    }
  }
  if (!result.successful) return result;

  std::lock_guard<std::mutex> lock(m_reconfig_mutex_);
  m_gain_value_dirty_ = m_gain_value_dirty_ || gain_v;
  m_gain_auto_dirty_ = m_gain_auto_dirty_ || gain_a;
  m_exposure_value_dirty_ = m_exposure_value_dirty_ || exp_v;
  m_exposure_auto_dirty_ = m_exposure_auto_dirty_ || exp_a;
  m_resolution_dirty_ = m_resolution_dirty_ || res;
  m_frame_rate_dirty_ = m_frame_rate_dirty_ || fps;
  return result;
}

void ArenaCameraNode::apply_pending_reconfig_()
{
  bool gain_v, gain_a, exp_v, exp_a, res, fps;
  {
    std::lock_guard<std::mutex> lock(m_reconfig_mutex_);
    gain_v = m_gain_value_dirty_;
    gain_a = m_gain_auto_dirty_;
    exp_v = m_exposure_value_dirty_;
    exp_a = m_exposure_auto_dirty_;
    res = m_resolution_dirty_;
    fps = m_frame_rate_dirty_;
    m_gain_value_dirty_ = false;
    m_gain_auto_dirty_ = false;
    m_exposure_value_dirty_ = false;
    m_exposure_auto_dirty_ = false;
    m_resolution_dirty_ = false;
    m_frame_rate_dirty_ = false;
  }
  if (!gain_v && !gain_a && !exp_v && !exp_a && !res && !fps) return;

  try {
    if (gain_v || gain_a) {
      gain_ = this->get_parameter("gain").as_double();
      is_passed_gain_ = gain_ >= 0;
      gain_auto_ = this->get_parameter("gain_auto").as_string();
      // Setting a concrete gain value without also choosing a mode means
      // "go manual" -- matches `ros2 param set <node> gain 13.0`.
      if (gain_v && !gain_a && is_passed_gain_) {
        gain_auto_ = "Off";
      }
      set_nodes_gain_();  // Gain/GainAuto are writable while streaming
    }

    if (exp_v || exp_a) {
      exposure_time_ = this->get_parameter("exposure_time").as_double();
      is_passed_exposure_time_ = exposure_time_ >= 0;
      exposure_auto_ = this->get_parameter("exposure_auto").as_string();
      if (exp_v && !exp_a && is_passed_exposure_time_) {
        exposure_auto_ = "Off";
      }
      set_nodes_exposure_();  // ExposureTime/ExposureAuto writable while streaming
      // Exposure bounds the achievable rate, so re-assert the cap after it.
      set_nodes_frame_rate_();
    }

    if (fps) {
      frame_rate_ = this->get_parameter("frame_rate").as_double();
      is_passed_frame_rate_ = frame_rate_ > 0;
      set_nodes_frame_rate_();
    }

    if (res) {
      const int64_t w = this->get_parameter("width").as_int();
      const int64_t h = this->get_parameter("height").as_int();
      // Width/Height are NOT writable while streaming: stop, set, restart.
      m_pDevice->StopStream();
      width_ = static_cast<size_t>(w);
      is_passed_width = true;
      height_ = static_cast<size_t>(h);
      is_passed_height = true;
      set_nodes_roi_();
      m_pDevice->StartStream();
      log_info(std::string("\tResolution changed to ") + std::to_string(w) +
               "x" + std::to_string(h) + " (stream restarted)");
    }
  } catch (GenICam::GenericException& e) {
    log_err(std::string("apply_pending_reconfig failed (GenICam): ") + e.what());
  } catch (std::exception& e) {
    log_err(std::string("apply_pending_reconfig failed: ") + e.what());
  }
}

// just for debugging
void ArenaCameraNode::set_nodes_test_pattern_image_()
{
  auto nodemap = m_pDevice->GetNodeMap();
  Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TestPattern", "Pattern3");
}
