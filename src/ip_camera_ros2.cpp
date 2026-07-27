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

  if (enable_cam_info_ && correct_cam_info_) {
    cam_info_msg_ = ip_camera_ros2::build_camera_info(
      calibration_, frame_, this->get_clock()->now());
    const std::string cam_info_topic = ip_camera_ros2::derive_camera_info_topic(image_topic_);
    cam_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(cam_info_topic, 10);
  }

  frame_buffer_ = std::make_unique<ip_camera_ros2::FrameBuffer>(buffer_size_);

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
  capturer_ = std::make_unique<RTSPCapturer>(url_, *frame_buffer_, this->get_logger());
  capturer_thread_ = std::thread([this]() {
        capturer_->run();
    });

  RCLCPP_INFO(this->get_logger(), "Activating %s node", this->get_name());
  return CallbackReturn::SUCCESS;
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
  cam_info_pub_.reset();
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
  cv::Mat local_frame;
  if (!frame_buffer_->pop_latest(local_frame)) {
    RCLCPP_DEBUG(this->get_logger(), "No frames available in buffer");
    return;
  }

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

  // Stamp and publish
  const rclcpp::Time stamp = this->get_clock()->now();
  image_msg_.header.stamp = stamp;
  image_msg_.image = local_frame;

  // Fill and publish by unique_ptr instead of by value: avoids an extra Image copy and
  // allows zero-copy delivery to intra-process subscribers.
  auto image_msg = std::make_unique<sensor_msgs::msg::Image>();
  image_msg_.toImageMsg(*image_msg);
  image_pub_->publish(std::move(image_msg));

  if (enable_cam_info_ && correct_cam_info_) {
    cam_info_msg_.header.stamp = stamp;
    // Always reflect the dimensions of the frame actually being published: they may
    // differ from image_width_/image_height_ (unset, or clamped by the crop ROI check).
    cam_info_msg_.height = static_cast<uint32_t>(local_frame.rows);
    cam_info_msg_.width = static_cast<uint32_t>(local_frame.cols);
    cam_info_pub_->publish(std::make_unique<sensor_msgs::msg::CameraInfo>(cam_info_msg_));
  }
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

  declare_and_get("enable_cam_info", false, "Enable camera info publishing", enable_cam_info_);

  if (enable_cam_info_) {
    declare_and_get(
      "distortion_model", std::string(),
      "Camera distortion model, e.g. 'plumb_bob' or 'equidistant'", calibration_.distortion_model);

    declare_and_get_array(
      "camera_matrix", "3x3 camera intrinsic matrix K (row-major, 9 elements)", calibration_.k);
    declare_and_get_array(
      "distortion_coefficients", "Distortion coefficients D (size depends on distortion_model)",
      calibration_.d);
    declare_and_get_array(
      "rectification_matrix", "3x3 rectification matrix R (row-major, 9 elements)",
      calibration_.r);
    declare_and_get_array(
      "projection_matrix", "3x4 projection matrix P (row-major, 12 elements)", calibration_.p);

    correct_cam_info_ = ip_camera_ros2::is_calibration_valid(calibration_);
    if (!correct_cam_info_) {
      RCLCPP_WARN(this->get_logger(), "Calibration matrix sizes are incorrect");
    } else {
      RCLCPP_INFO(this->get_logger(), "Calibration camera values loaded correctly");
    }
  }
}
