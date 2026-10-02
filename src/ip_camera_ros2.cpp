// Copyright (c) 2024 Óscar Pons Fernández
// Copyright (c) 2024 Grupo Avispa, DTE, Universidad de Málaga
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ip_camera_ros2/ip_camera_ros2.hpp"

// C++
#include <chrono>
#include <iomanip>
#include <sstream>
#include <utility>

// ROS 2
#include "sensor_msgs/image_encodings.hpp"

#include "ip_camera_ros2/image_ops.hpp"
#include "ip_camera_ros2/rtsp_capturer.hpp"
#include "ip_camera_ros2/topic_utils.hpp"

IpCameraRos2::IpCameraRos2(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("ip_camera_ros2", options)
{
}

IpCameraRos2::~IpCameraRos2()
{
  stop_capture();
}

CallbackReturn IpCameraRos2::on_configure(const rclcpp_lifecycle::State &)
{
  update_params();

  image_msg_ = initialize_image_msg();

  cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(image_topic_, 10);

  // JPEG-compressed counterpart of image_pub_, published lazily (only while subscribed)
  // to avoid the encoding cost when nobody is listening.
  image_compressed_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
    image_topic_ + "/compressed", 10);

  // Always created: the flag decides whether it is used, so camera info can be enabled later.
  cam_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
    ip_camera_ros2::derive_camera_info_topic(image_topic_), 10);

  set_camera_info_srv_ = this->create_service<sensor_msgs::srv::SetCameraInfo>(
    ip_camera_ros2::derive_set_camera_info_service(image_topic_),
    std::bind(
      &IpCameraRos2::handle_set_camera_info, this, std::placeholders::_1, std::placeholders::_2));

  post_set_handle_ = this->add_post_set_parameters_callback(
    std::bind(&IpCameraRos2::on_parameters_set, this, std::placeholders::_1));

  frame_buffer_ = std::make_unique<ip_camera_ros2::FrameBuffer>(buffer_size_);
  last_published_stamp_.reset();

  RCLCPP_INFO(this->get_logger(), "Configured %s node", this->get_name());
  return CallbackReturn::SUCCESS;
}

CallbackReturn IpCameraRos2::on_activate(const rclcpp_lifecycle::State & state)
{
  // Activates image_pub_/cam_info_pub_, registered as managed entities by create_publisher().
  LifecycleNode::on_activate(state);

  const unsigned int frame_period_ms = static_cast<unsigned int>(1000 / frame_rate_);
  RCLCPP_INFO(this->get_logger(), "Frame period set to: %u ms", frame_period_ms);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(frame_period_ms),
    std::bind(&IpCameraRos2::capture_ipcam_image, this),
    cb_group_);

  // Only connect to the RTSP stream once active, and release it on deactivation.
  start_capture();

  RCLCPP_INFO(this->get_logger(), "Activating %s node", this->get_name());
  return CallbackReturn::SUCCESS;
}

void IpCameraRos2::start_capture()
{
  capturer_ = std::make_unique<RTSPCapturer>(
    url_, *frame_buffer_, this->get_logger(), this->get_clock());
  capturer_thread_ = std::thread([this]() {
        capturer_->run();
    });
}

CallbackReturn IpCameraRos2::on_deactivate(const rclcpp_lifecycle::State & state)
{
  stop_capture();

  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }

  // Deactivates image_pub_/cam_info_pub_: any straggling publish() becomes a safe no-op.
  LifecycleNode::on_deactivate(state);

  RCLCPP_INFO(this->get_logger(), "Deactivating %s node", this->get_name());
  return CallbackReturn::SUCCESS;
}

CallbackReturn IpCameraRos2::on_cleanup(const rclcpp_lifecycle::State &)
{
  frame_buffer_.reset();
  image_pub_.reset();
  image_compressed_pub_.reset();
  cam_info_pub_.reset();
  set_camera_info_srv_.reset();
  post_set_handle_.reset();
  cb_group_.reset();

  RCLCPP_INFO(this->get_logger(), "Cleaning up %s node", this->get_name());
  return CallbackReturn::SUCCESS;
}

CallbackReturn IpCameraRos2::on_shutdown(const rclcpp_lifecycle::State & state)
{
  // Shutdown can be requested from any state, so tear down defensively.
  stop_capture();
  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }
  frame_buffer_.reset();
  image_pub_.reset();
  image_compressed_pub_.reset();
  cam_info_pub_.reset();

  RCLCPP_INFO(
    this->get_logger(), "Shutting down %s node from state %s", this->get_name(),
    state.label().c_str());
  return CallbackReturn::SUCCESS;
}

void IpCameraRos2::stop_capture()
{
  if (capturer_) {
    capturer_->stop();
  }
  if (capturer_thread_.joinable()) {
    capturer_thread_.join();
  }
  capturer_.reset();
}

void IpCameraRos2::capture_ipcam_image()
{
  ip_camera_ros2::StampedFrame captured;
  if (!frame_buffer_->pop_latest(captured)) {
    RCLCPP_DEBUG(this->get_logger(), "No frames available in buffer");
    return;
  }

  // Never publish the same capture twice
  if (last_published_stamp_ && captured.stamp == *last_published_stamp_) {
    return;
  }

  cv::Mat local_frame = captured.image;

  // Crop or resize
  const int orig_cols = local_frame.cols;
  const int orig_rows = local_frame.rows;
  local_frame = ip_camera_ros2::apply_crop_or_resize(
    local_frame, ip_camera_ros2::CropRegion{image_width_, image_height_, offset_x_, offset_y_});
  if (local_frame.empty()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Crop ROI (%d,%d,%d,%d) is outside the frame bounds (%dx%d); skipping this frame",
      offset_x_, offset_y_, image_width_, image_height_, orig_cols, orig_rows);
    return;
  }

  // Stamp with the arrival of the frame, corrected by the configured constant latency
  const rclcpp::Time stamp = captured.stamp - rclcpp::Duration::from_seconds(stamp_offset_);
  last_published_stamp_ = captured.stamp;
  image_msg_.header.stamp = stamp;
  image_msg_.image = local_frame;

  // Fill and publish by unique_ptr instead of by value: avoids an extra Image copy and
  // allows zero-copy delivery to intra-process subscribers.
  auto image_msg = std::make_unique<sensor_msgs::msg::Image>();
  image_msg_.toImageMsg(*image_msg);
  image_pub_->publish(std::move(image_msg));

  // Only encode when someone is actually subscribed: JPEG encoding is comparatively
  // expensive, so it is skipped unless a viewer (e.g. rqt_image_view) is listening.
  if (image_compressed_pub_->get_subscription_count() > 0) {
    auto compressed_msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
    image_msg_.toCompressedImageMsg(*compressed_msg);
    image_compressed_pub_->publish(std::move(compressed_msg));
  }

  auto cam_info = make_camera_info(stamp, local_frame.rows, local_frame.cols);
  if (cam_info) {
    cam_info_pub_->publish(std::make_unique<sensor_msgs::msg::CameraInfo>(std::move(*cam_info)));
  }
}

std::optional<sensor_msgs::msg::CameraInfo> IpCameraRos2::make_camera_info(
  const rclcpp::Time & stamp, int rows, int cols)
{
  std::string stale_calibration;
  std::optional<sensor_msgs::msg::CameraInfo> cam_info;
  {
    std::lock_guard<std::mutex> lock(cam_info_mutex_);
    if (!enable_cam_info_ || !correct_cam_info_) {
      return std::nullopt;
    }
    if ((calibration_width_ > 0 && cols != calibration_width_) ||
      (calibration_height_ > 0 && rows != calibration_height_))
    {
      stale_calibration = "The published image is " + std::to_string(cols) + "x" +
        std::to_string(rows) + " but the camera was calibrated at " +
        std::to_string(calibration_width_) + "x" + std::to_string(calibration_height_);
    } else {
      cam_info = cam_info_msg_;
      cam_info->header.stamp = stamp;
      // Always reflect the dimensions of the frame actually being published: they may
      // differ from image_width_/image_height_ (unset, or clamped by the crop ROI check).
      cam_info->height = static_cast<uint32_t>(rows);
      cam_info->width = static_cast<uint32_t>(cols);
    }
  }
  if (!stale_calibration.empty()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 10000,
      "Not publishing camera info: %s. The intrinsics K are in pixels of the calibrated image, "
      "so a change of image_width, image_height, offset_x, offset_y or of the stream resolution "
      "invalidates them. Calibrate again or restore the previous size",
      stale_calibration.c_str());
  }
  return cam_info;
}

void IpCameraRos2::refresh_calibration()
{
  bool enable = false;
  ip_camera_ros2::CalibrationParams calibration;
  int calibration_width = -1;
  int calibration_height = -1;
  this->get_parameter("enable_cam_info", enable);
  this->get_parameter("distortion_model", calibration.distortion_model);
  this->get_parameter("camera_matrix", calibration.k);
  this->get_parameter("distortion_coefficients", calibration.d);
  this->get_parameter("rectification_matrix", calibration.r);
  this->get_parameter("projection_matrix", calibration.p);
  this->get_parameter("calibration_width", calibration_width);
  this->get_parameter("calibration_height", calibration_height);

  const ip_camera_ros2::CalibrationCheck check = ip_camera_ros2::check_calibration(calibration);
  {
    std::lock_guard<std::mutex> lock(cam_info_mutex_);
    enable_cam_info_ = enable;
    calibration_ = calibration;
    calibration_width_ = calibration_width;
    calibration_height_ = calibration_height;
    correct_cam_info_ = check.valid;
    if (check.valid) {
      cam_info_msg_ = ip_camera_ros2::build_camera_info(
        calibration_, frame_, this->get_clock()->now());
    }
  }

  if (!enable) {
    return;
  }
  if (!check.valid) {
    RCLCPP_WARN(
      this->get_logger(), "Camera info is enabled but not published: %s", check.reason.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "Calibration camera values loaded correctly");
    if (!check.warning.empty()) {
      RCLCPP_WARN(this->get_logger(), "Calibration: %s", check.warning.c_str());
    }
  }
}

void IpCameraRos2::on_parameters_set(const std::vector<rclcpp::Parameter> & parameters)
{
  static const std::vector<std::string> calibration_parameters = {
    "enable_cam_info", "distortion_model", "camera_matrix", "distortion_coefficients",
    "rectification_matrix", "projection_matrix", "calibration_width", "calibration_height"};
  for (const auto & parameter : parameters) {
    for (const auto & name : calibration_parameters) {
      if (parameter.get_name() == name) {
        refresh_calibration();
        return;
      }
    }
  }
}

namespace
{

/**
 * @brief Format a list of doubles as a YAML flow sequence of floating-point numbers.
 *
 * Every element keeps a decimal point, since a parameter file that holds an integer where a
 * double array is expected is rejected on load.
 *
 * @param values Values to format.
 * @return The sequence, such as `[1.0, 0.5]`.
 */
std::string to_yaml_sequence(const std::vector<double> & values)
{
  std::ostringstream stream;
  stream << std::setprecision(12) << "[";
  for (size_t i = 0; i < values.size(); ++i) {
    std::ostringstream element;
    element << std::setprecision(12) << values[i];
    std::string text = element.str();
    if (text.find_first_of(".eEn") == std::string::npos) {
      text += ".0";
    }
    stream << (i == 0 ? "" : ", ") << text;
  }
  stream << "]";
  return stream.str();
}

}  // namespace

void IpCameraRos2::handle_set_camera_info(
  const std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Request> request,
  std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Response> response)
{
  const sensor_msgs::msg::CameraInfo & info = request->camera_info;
  ip_camera_ros2::CalibrationParams calibration;
  calibration.distortion_model = info.distortion_model;
  calibration.k.assign(info.k.begin(), info.k.end());
  calibration.d = info.d;
  calibration.r.assign(info.r.begin(), info.r.end());
  calibration.p.assign(info.p.begin(), info.p.end());

  const ip_camera_ros2::CalibrationCheck check = ip_camera_ros2::check_calibration(calibration);
  if (!check.valid) {
    response->success = false;
    response->status_message = check.reason;
    RCLCPP_WARN(this->get_logger(), "Rejected the calibration: %s", check.reason.c_str());
    return;
  }

  // One atomic update, so the calibration is rebuilt once and never from a mix of old and new
  const auto result = this->set_parameters_atomically(
  {
    rclcpp::Parameter("distortion_model", calibration.distortion_model),
    rclcpp::Parameter("camera_matrix", calibration.k),
    rclcpp::Parameter("distortion_coefficients", calibration.d),
    rclcpp::Parameter("rectification_matrix", calibration.r),
    rclcpp::Parameter("projection_matrix", calibration.p),
    rclcpp::Parameter("calibration_width", static_cast<int64_t>(info.width)),
    rclcpp::Parameter("calibration_height", static_cast<int64_t>(info.height)),
    rclcpp::Parameter("enable_cam_info", true),
    });
  if (!result.successful) {
    response->success = false;
    response->status_message = "Could not set the calibration parameters: " + result.reason;
    return;
  }

  response->success = true;
  response->status_message = "Calibration applied";
  RCLCPP_INFO(
    this->get_logger(),
    "Calibration applied. To persist it, copy these values to the parameters file:\n"
    "%s:\n  ros__parameters:\n    enable_cam_info: true\n    calibration_width: %u\n"
    "    calibration_height: %u\n    distortion_model: %s\n    camera_matrix: %s\n"
    "    distortion_coefficients: %s\n    rectification_matrix: %s\n"
    "    projection_matrix: %s",
    this->get_name(), info.width, info.height, calibration.distortion_model.c_str(),
    to_yaml_sequence(calibration.k).c_str(), to_yaml_sequence(calibration.d).c_str(),
    to_yaml_sequence(calibration.r).c_str(), to_yaml_sequence(calibration.p).c_str());
}

cv_bridge::CvImage IpCameraRos2::initialize_image_msg()
{
  cv_bridge::CvImage image_msg;
  image_msg.header.frame_id = frame_;
  image_msg.encoding = sensor_msgs::image_encodings::BGR8;
  return image_msg;
}

void IpCameraRos2::update_params()
{
  declare_and_get(
    "image_topic", std::string("/image"),
    "Topic of the ip camera image. When enable_cam_info is true, CameraInfo is "
    "published on the standard sibling topic (e.g. '/image' -> '/camera_info')",
    image_topic_);

  declare_and_get(
    "image_height", -1, "Target image height after resize or crop (-1 = disabled)",
    image_height_);
  declare_and_get(
    "image_width", -1, "Target image width after resize or crop (-1 = disabled)", image_width_);
  declare_and_get(
    "offset_x", -1, "Crop image top-left corner offset X (-1 = disabled)", offset_x_);
  declare_and_get(
    "offset_y", -1, "Crop image top-left corner offset Y (-1 = disabled)", offset_y_);

  // The URL may embed credentials (e.g. rtsp://user:pass@host): never log its value.
  declare_parameter_if_not_declared(
    this, "url", rclcpp::ParameterValue("ipcam_url"),
    rcl_interfaces::msg::ParameterDescriptor().set__description("IP Cam url"));
  this->get_parameter("url", url_);

  declare_and_get(
    "tf", std::string("camera_optical_link"), "Camera frame id", frame_);

  declare_and_get(
    "stamp_offset", 0.0,
    "Seconds subtracted from the time a frame reaches the host to compensate the constant "
    "latency of the camera and the stream",
    stamp_offset_);

  declare_and_get(
    "frame_rate", 30,
    "Frame rate for publishing images, in Hz (values below 1 are clamped to 1)", frame_rate_);
  if (frame_rate_ < 1) {
    RCLCPP_WARN(
      this->get_logger(),
      "The parameter frame_rate must be a positive integer, got [%d]. Falling back to 1",
      frame_rate_);
    frame_rate_ = 1;
  }

  // buffer_size_ is size_t: kept out of declare_and_get<T>, whose default value and output
  // must share the same type. It is read into an int because rclcpp stores integer
  // parameters as int64_t and cannot deduce an unsigned type; going through a signed
  // value also lets the range check below reject a negative setting, which converting
  // straight to size_t would have turned into an enormous buffer.
  declare_parameter_if_not_declared(
    this, "buffer_size", rclcpp::ParameterValue(2),
    rcl_interfaces::msg::ParameterDescriptor().set__description(
      "Maximum number of frames the producer thread accumulates before the consumer "
      "drains them. The consumer always keeps only the latest frame and discards any "
      "backlog, so this only bounds worst-case memory usage, not smoothing/latency."));
  int buffer_size = 2;
  this->get_parameter("buffer_size", buffer_size);
  if (buffer_size < 1) {
    RCLCPP_WARN(this->get_logger(), "The parameter buffer_size must be at least 1; using 1");
    buffer_size = 1;
  }
  buffer_size_ = static_cast<size_t>(buffer_size);
  RCLCPP_INFO(this->get_logger(), "The parameter buffer_size is set to: [%zu]", buffer_size_);

  // The calibration parameters are always declared and can change at runtime, so that a
  // calibration tool can apply its result through the set_camera_info service.
  declare_parameter_if_not_declared(
    this, "enable_cam_info", rclcpp::ParameterValue(false),
    rcl_interfaces::msg::ParameterDescriptor().set__description(
      "Enable camera info publishing. Only valid calibrations are published"));
  declare_parameter_if_not_declared(
    this, "distortion_model", rclcpp::ParameterValue(std::string()),
    rcl_interfaces::msg::ParameterDescriptor().set__description(
      "Camera distortion model, e.g. 'plumb_bob' or 'equidistant'"));
  declare_and_get_array(
    "camera_matrix", "3x3 camera intrinsic matrix K (row-major, 9 elements)", calibration_.k);
  declare_and_get_array(
    "distortion_coefficients", "Distortion coefficients D (size depends on distortion_model)",
    calibration_.d);
  declare_and_get_array(
    "rectification_matrix", "3x3 rectification matrix R (row-major, 9 elements)", calibration_.r);
  declare_and_get_array(
    "projection_matrix", "3x4 projection matrix P (row-major, 12 elements)", calibration_.p);
  declare_parameter_if_not_declared(
    this, "calibration_width", rclcpp::ParameterValue(-1),
    rcl_interfaces::msg::ParameterDescriptor().set__description(
      "Width of the image the camera was calibrated with (-1 = do not check). Camera info is "
      "not published if the published image has another width"));
  declare_parameter_if_not_declared(
    this, "calibration_height", rclcpp::ParameterValue(-1),
    rcl_interfaces::msg::ParameterDescriptor().set__description(
      "Height of the image the camera was calibrated with (-1 = do not check). Camera info is "
      "not published if the published image has another height"));

  refresh_calibration();
}
