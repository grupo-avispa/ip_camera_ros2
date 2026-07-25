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

IpCameraRos2::IpCameraRos2()
: Node("ip_camera_ros2")
{
  update_params();

  image_msg_ = initialize_image_msg();

  cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // CameraPublisher bundles image + CameraInfo atomically under the standard
  // "<image_topic>/camera_info" sibling naming; plain Publisher is used when no valid
  // calibration is available, since CameraPublisher always requires both messages.
  if (enable_cam_info_ && correct_cam_info_) {
    cam_info_msg_ = ip_camera_ros2::build_camera_info(
      calibration_, frame_, this->get_clock()->now());
    cam_pub_ = image_transport::create_camera_publisher(this, image_topic_);
  } else {
    image_pub_ = image_transport::create_publisher(this, image_topic_);
  }

  const unsigned int frame_period_ms = static_cast<unsigned int>(1000 / frame_rate_);
  RCLCPP_INFO(this->get_logger(), "Frame period set to: %u ms", frame_period_ms);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(frame_period_ms),
    std::bind(&IpCameraRos2::capture_ipcam_image, this),
    cb_group_);

  // Own the frame buffer and the capturer, and run the latter in a dedicated producer
  // thread: this node is the single owner of both, instead of exposing them as public
  // fields for main() to wire up externally.
  frame_buffer_ = std::make_unique<ip_camera_ros2::FrameBuffer>(buffer_size_);
  capturer_ = std::make_unique<RTSPCapturer>(url_, *frame_buffer_, this->get_logger());
  capturer_thread_ = std::thread([this]() {
      capturer_->run();
    });
}

IpCameraRos2::~IpCameraRos2()
{
  // Stop the producer thread before the automatic destruction of capturer_/frame_buffer_
  // (in reverse declaration order) runs; std::thread's destructor would otherwise call
  // std::terminate() on a still-joinable thread.
  capturer_->stop();
  if (capturer_thread_.joinable()) {
    capturer_thread_.join();
  }
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

  // Publish by shared_ptr instead of by value: avoids an extra Image copy and allows
  // zero-copy delivery to intra-process subscribers.
  if (enable_cam_info_ && correct_cam_info_) {
    cam_info_msg_.header.stamp = stamp;
    // Always reflect the dimensions of the frame actually being published: they may
    // differ from image_width_/image_height_ (unset, or clamped by the crop ROI check).
    cam_info_msg_.height = static_cast<uint32_t>(local_frame.rows);
    cam_info_msg_.width = static_cast<uint32_t>(local_frame.cols);
    cam_pub_.publish(
      image_msg_.toImageMsg(), std::make_shared<sensor_msgs::msg::CameraInfo>(cam_info_msg_));
  } else {
    image_pub_.publish(image_msg_.toImageMsg());
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
  // must share the same type.
  declare_parameter_if_not_declared(
    this, "buffer_size", rclcpp::ParameterValue(2),
    rcl_interfaces::msg::ParameterDescriptor().set__description(
      "Maximum number of frames the producer thread accumulates before the consumer "
      "drains them. The consumer always keeps only the latest frame and discards any "
      "backlog, so this only bounds worst-case memory usage, not smoothing/latency."));
  this->get_parameter("buffer_size", buffer_size_);
  if (buffer_size_ < 1) {
    RCLCPP_WARN(this->get_logger(), "The parameter buffer_size must be at least 1; using 1");
    buffer_size_ = 1;
  }
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
