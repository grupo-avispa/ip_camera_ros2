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
#include <algorithm>
#include <chrono>

// ROS 2
#include "sensor_msgs/image_encodings.hpp"

#include "ip_camera_ros2/rtsp_capturer.hpp"

IpCameraRos2::IpCameraRos2()
: Node("ip_camera_ros2")
{
  update_params();

  image_msg_ = initialize_image_msg();

  cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  image_pub_ = image_transport::create_publisher(this, image_topic_);

  const unsigned int frame_period_ms = static_cast<unsigned int>(1000 / frame_rate_);
  RCLCPP_INFO(this->get_logger(), "Frame period set to: %u ms", frame_period_ms);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(frame_period_ms),
    std::bind(&IpCameraRos2::capture_ipcam_image, this),
    cb_group_);

  if (enable_cam_info_ && correct_cam_info_) {
    cam_info_msg_ = create_cam_info_msg(this->get_clock()->now());
    cam_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(cam_info_topic_, 10);
  }
}

void IpCameraRos2::capture_ipcam_image()
{
  cv::Mat local_frame;

  // Consumer: grab the latest frame and discard any stale backlog
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    if (frame_buffer_.empty()) {
      RCLCPP_DEBUG(this->get_logger(), "No frames available in buffer");
      return;
    }
    local_frame = std::move(frame_buffer_.back());
    frame_buffer_.clear();
  }

  // Crop or resize
  if (image_width_ > 0 && image_height_ > 0) {
    if (offset_x_ >= 0 && offset_y_ >= 0) {
      const cv::Rect frame_rect(0, 0, local_frame.cols, local_frame.rows);
      const cv::Rect roi = cv::Rect(offset_x_, offset_y_, image_width_, image_height_) & frame_rect;
      if (roi.width == 0 || roi.height == 0) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "Crop ROI (%d,%d,%d,%d) is outside the frame bounds (%dx%d); skipping this frame",
          offset_x_, offset_y_, image_width_, image_height_, local_frame.cols, local_frame.rows);
        return;
      }
      local_frame = local_frame(roi).clone();
    } else {
      cv::resize(local_frame, local_frame, cv::Size(image_width_, image_height_));
    }
  }

  // Stamp and publish
  const rclcpp::Time stamp = this->get_clock()->now();
  image_msg_.header.stamp = stamp;
  image_msg_.image = local_frame;

  if (enable_cam_info_ && correct_cam_info_) {
    cam_info_msg_.header.stamp = stamp;
    // Always reflect the dimensions of the frame actually being published: they may
    // differ from image_width_/image_height_ (unset, or clamped by the crop ROI check).
    cam_info_msg_.height = static_cast<uint32_t>(local_frame.rows);
    cam_info_msg_.width = static_cast<uint32_t>(local_frame.cols);
    cam_info_pub_->publish(cam_info_msg_);
  }

  image_pub_.publish(*image_msg_.toImageMsg());
}

cv_bridge::CvImage IpCameraRos2::initialize_image_msg()
{
  cv_bridge::CvImage image_msg;
  image_msg.header.frame_id = frame_;
  image_msg.encoding = sensor_msgs::image_encodings::BGR8;
  return image_msg;
}

sensor_msgs::msg::CameraInfo IpCameraRos2::create_cam_info_msg(rclcpp::Time stamp)
{
  sensor_msgs::msg::CameraInfo cam_info_msg;
  cam_info_msg.header.frame_id = frame_;
  cam_info_msg.header.stamp = stamp;
  // height/width are filled in per-message from the actually published frame, since they
  // may differ from image_width_/image_height_ (unset, or clamped by the crop ROI check).
  cam_info_msg.distortion_model = distortion_model_;
  std::copy_n(k_.begin(), std::min(k_.size(), cam_info_msg.k.size()), cam_info_msg.k.begin());
  std::copy_n(p_.begin(), std::min(p_.size(), cam_info_msg.p.size()), cam_info_msg.p.begin());
  std::copy_n(r_.begin(), std::min(r_.size(), cam_info_msg.r.size()), cam_info_msg.r.begin());
  cam_info_msg.d = d_;
  return cam_info_msg;
}

void IpCameraRos2::update_params()
{
  declare_parameter_if_not_declared(
    this, "image_topic", rclcpp::ParameterValue("/image"),
    rcl_interfaces::msg::ParameterDescriptor().set__description(
      "Topic of the ip camera image"));
  this->get_parameter("image_topic", image_topic_);
  RCLCPP_INFO(this->get_logger(), "The parameter image_topic_ is set to: [%s]",
    image_topic_.c_str());

  declare_parameter_if_not_declared(
    this, "cam_info_topic", rclcpp::ParameterValue("/camera_info"),
    rcl_interfaces::msg::ParameterDescriptor().set__description(
      "Topic of the ip camera info"));
  this->get_parameter("cam_info_topic", cam_info_topic_);
  RCLCPP_INFO(
    this->get_logger(), "The parameter cam_info_topic is set to: [%s]", cam_info_topic_.c_str());

  declare_parameter_if_not_declared(
    this, "image_height", rclcpp::ParameterValue(-1),
    rcl_interfaces::msg::ParameterDescriptor().set__description("Depth image height"));
  this->get_parameter("image_height", image_height_);
  RCLCPP_INFO(this->get_logger(), "The parameter image_height is set to: [%d]", image_height_);

  declare_parameter_if_not_declared(
    this, "image_width", rclcpp::ParameterValue(-1),
    rcl_interfaces::msg::ParameterDescriptor().set__description("Depth image width"));
  this->get_parameter("image_width", image_width_);
  RCLCPP_INFO(this->get_logger(), "The parameter image_width is set to: [%d]", image_width_);

  declare_parameter_if_not_declared(
    this, "offset_x", rclcpp::ParameterValue(-1),
    rcl_interfaces::msg::ParameterDescriptor().set__description("Crop image offset X"));
  this->get_parameter("offset_x", offset_x_);
  RCLCPP_INFO(this->get_logger(), "The parameter offset_x is set to: [%d]", offset_x_);

  declare_parameter_if_not_declared(
    this, "offset_y", rclcpp::ParameterValue(-1),
    rcl_interfaces::msg::ParameterDescriptor().set__description("Crop image offset Y"));
  this->get_parameter("offset_y", offset_y_);
  RCLCPP_INFO(this->get_logger(), "The parameter offset_y is set to: [%d]", offset_y_);

  declare_parameter_if_not_declared(
    this, "url", rclcpp::ParameterValue("ipcam_url"),
    rcl_interfaces::msg::ParameterDescriptor().set__description("IP Cam url"));
  this->get_parameter("url", url_);

  declare_parameter_if_not_declared(
    this, "tf", rclcpp::ParameterValue("camera_optical_link"),
    rcl_interfaces::msg::ParameterDescriptor().set__description("Camera frame id"));
  this->get_parameter("tf", frame_);
  RCLCPP_INFO(this->get_logger(), "The parameter frame is set to: [%s]", frame_.c_str());

  declare_parameter_if_not_declared(
    this, "frame_rate", rclcpp::ParameterValue(30),
    rcl_interfaces::msg::ParameterDescriptor().set__description(
      "Frame rate for publishing images"));
  this->get_parameter("frame_rate", frame_rate_);
  if (frame_rate_ < 1) {
    RCLCPP_WARN(
      this->get_logger(),
      "The parameter frame_rate must be a positive integer, got [%d]. Falling back to 1",
      frame_rate_);
    frame_rate_ = 1;
  }
  RCLCPP_INFO(this->get_logger(), "The parameter frame_rate is set to: [%d]", frame_rate_);

  declare_parameter_if_not_declared(
    this, "buffer_size", rclcpp::ParameterValue(30),
    rcl_interfaces::msg::ParameterDescriptor().set__description(
      "Frame buffer size for the producer-consumer pattern"));
  this->get_parameter("buffer_size", buffer_size_);
  RCLCPP_INFO(
    this->get_logger(), "The parameter buffer_size is set to: [%zu]", buffer_size_);

  declare_parameter_if_not_declared(
    this, "enable_cam_info", rclcpp::ParameterValue(false),
    rcl_interfaces::msg::ParameterDescriptor().set__description(
      "Enable camera info publishing"));
  this->get_parameter("enable_cam_info", enable_cam_info_);

  if (enable_cam_info_) {
    RCLCPP_INFO(this->get_logger(), "The parameter enable_cam_info is set to: [true]");

    declare_parameter_if_not_declared(
      this, "distortion_model", rclcpp::ParameterValue("distortion_model"),
      rcl_interfaces::msg::ParameterDescriptor().set__description(
        "Camera calibration parameter"));
    this->get_parameter("distortion_model", distortion_model_);
    RCLCPP_INFO(
      this->get_logger(), "The parameter distortion_model is set to: [%s]",
      distortion_model_.c_str());

    declare_parameter_if_not_declared(
      this, "camera_matrix", rclcpp::PARAMETER_DOUBLE_ARRAY,
      rcl_interfaces::msg::ParameterDescriptor().set__description(
        "Camera calibration parameter"));
    this->get_parameter("camera_matrix", k_);

    declare_parameter_if_not_declared(
      this, "distortion_coefficients", rclcpp::PARAMETER_DOUBLE_ARRAY,
      rcl_interfaces::msg::ParameterDescriptor().set__description(
        "Camera calibration parameter"));
    this->get_parameter("distortion_coefficients", d_);

    declare_parameter_if_not_declared(
      this, "rectification_matrix", rclcpp::PARAMETER_DOUBLE_ARRAY,
      rcl_interfaces::msg::ParameterDescriptor().set__description(
        "Camera calibration parameter"));
    this->get_parameter("rectification_matrix", r_);

    declare_parameter_if_not_declared(
      this, "projection_matrix", rclcpp::PARAMETER_DOUBLE_ARRAY,
      rcl_interfaces::msg::ParameterDescriptor().set__description(
        "Camera calibration parameter"));
    this->get_parameter("projection_matrix", p_);

    if (k_.size() != 9 || r_.size() != 9 || p_.size() != 12) {
      RCLCPP_WARN(this->get_logger(), "Calibration matrix sizes are incorrect");
    } else {
      RCLCPP_INFO(this->get_logger(), "Calibration camera values loaded correctly");
      correct_cam_info_ = true;
    }
  } else {
    RCLCPP_INFO(this->get_logger(), "The parameter enable_cam_info is set to: [false]");
  }
}
