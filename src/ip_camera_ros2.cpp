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
  declare_and_get(
    "image_topic", std::string("/image"), "Topic of the ip camera image", image_topic_);
  declare_and_get(
    "cam_info_topic", std::string("/camera_info"), "Topic of the ip camera info",
    cam_info_topic_);

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
      "Camera distortion model, e.g. 'plumb_bob' or 'equidistant'", distortion_model_);

    declare_and_get_array(
      "camera_matrix", "3x3 camera intrinsic matrix K (row-major, 9 elements)", k_);
    declare_and_get_array(
      "distortion_coefficients", "Distortion coefficients D (size depends on distortion_model)",
      d_);
    declare_and_get_array(
      "rectification_matrix", "3x3 rectification matrix R (row-major, 9 elements)", r_);
    declare_and_get_array(
      "projection_matrix", "3x4 projection matrix P (row-major, 12 elements)", p_);

    if (k_.size() != 9 || r_.size() != 9 || p_.size() != 12) {
      RCLCPP_WARN(this->get_logger(), "Calibration matrix sizes are incorrect");
    } else {
      RCLCPP_INFO(this->get_logger(), "Calibration camera values loaded correctly");
      correct_cam_info_ = true;
    }
  }
}
