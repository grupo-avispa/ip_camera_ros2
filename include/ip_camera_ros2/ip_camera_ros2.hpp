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

#ifndef IP_CAMERA_ROS2__IP_CAMERA_ROS2_HPP_
#define IP_CAMERA_ROS2__IP_CAMERA_ROS2_HPP_

// C++
#include <ios>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "opencv2/opencv.hpp"

// ROS 2
#include "cv_bridge/cv_bridge.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

// Package
#include "ip_camera_ros2/camera_info_builder.hpp"
#include "ip_camera_ros2/frame_buffer.hpp"

// Forward declarations
class RTSPCapturer;

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * @brief ROS2 lifecycle node that publishes frames received from an IP / RTSP camera.
 *
 * The RTSP stream is only connected while active: on_activate() creates the
 * RTSPCapturer and starts its producer thread (which fills a FrameBuffer), and
 * on_deactivate() stops and destroys them. The publishing timer's callback dequeues
 * the latest frame and publishes it as sensor_msgs/Image (and optionally CameraInfo).
 *
 * image_transport is intentionally not used: it does not support LifecycleNode on
 * ROS 2 Jazzy (see ros-perception/image_common#108, fixed for Rolling only in #352),
 * so plain rclcpp_lifecycle::LifecyclePublisher is used instead.
 */
class IpCameraRos2 : public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * @brief Construct the IpCameraRos2 node. Does not read parameters or open the
   *        stream yet; see on_configure()/on_activate().
   *
   * @param options Node options.
   */
  explicit IpCameraRos2(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Stop the capture thread (if still running) and destroy the node.
   */
  ~IpCameraRos2();

  /**
   * @brief Read parameters and create the publishers.
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Activate the publishers, open the RTSP stream and start publishing.
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Stop publishing, close the RTSP stream and deactivate the publishers.
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Release the publishers and parameter-derived state.
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Ensure the capture thread and publishers are released from any state.
   */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  /// RTSP stream URL
  std::string url_;

  /// Target publishing frame rate (fps)
  int frame_rate_;

  /// Maximum number of frames held in frame_buffer_
  size_t buffer_size_;

  /// Shared hand-off point between the capture thread and the publishing timer
  std::unique_ptr<ip_camera_ros2::FrameBuffer> frame_buffer_;

  /// Producer that fills frame_buffer_ from a dedicated thread; only alive while active
  std::unique_ptr<RTSPCapturer> capturer_;
  std::thread capturer_thread_;

  /**
   * @brief Stop and join the capture thread, if any is running.
   */
  void stop_capture();

  /**
   * @brief Declares static ROS2 parameter and sets it to a given value if it was not already declared.
   *
   * @param node A node in which given parameter to be declared
   * @param param_name The name of parameter
   * @param default_value Parameter value to initialize with
   * @param parameter_descriptor Parameter descriptor (optional)
  */
  template<typename NodeT>
  void declare_parameter_if_not_declared(
    NodeT node,
    const std::string & param_name,
    const rclcpp::ParameterValue & default_value,
    const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor =
    rcl_interfaces::msg::ParameterDescriptor())
  {
    if (!node->has_parameter(param_name)) {
      node->declare_parameter(param_name, default_value, parameter_descriptor);
    }
  }

  /**
   * @brief Declares static ROS2 parameter with given type if it was not already declared.
   *
   * @param node A node in which given parameter to be declared
   * @param param_name The name of parameter
   * @param param_type The type of parameter
   * @param parameter_descriptor Parameter descriptor (optional)
  */
  template<typename NodeT>
  void declare_parameter_if_not_declared(
    NodeT node,
    const std::string & param_name,
    const rclcpp::ParameterType & param_type,
    const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor =
    rcl_interfaces::msg::ParameterDescriptor())
  {
    if (!node->has_parameter(param_name)) {
      node->declare_parameter(param_name, param_type, parameter_descriptor);
    }
  }

  /**
   * @brief Declare (if needed), read and log a scalar parameter in a single call.
   *
   * @param param_name Name of the parameter.
   * @param default_value Default value used if the parameter was not already declared.
   * @param description Human-readable description of the parameter.
   * @param out Output variable to store the read value in.
   */
  template<typename T>
  void declare_and_get(
    const std::string & param_name,
    const T & default_value,
    const std::string & description,
    T & out)
  {
    declare_parameter_if_not_declared(
      this, param_name, rclcpp::ParameterValue(default_value),
      rcl_interfaces::msg::ParameterDescriptor().set__description(description));
    this->get_parameter(param_name, out);
    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "The parameter " << param_name << " is set to: [" << std::boolalpha << out << "]");
  }

  /**
   * @brief Declare (if needed) and read a double-array parameter with no default value.
   *
   * Used for camera calibration parameters, which have no sensible default and must be
   * supplied by the user when camera info publishing is enabled.
   *
   * @param param_name Name of the parameter.
   * @param description Human-readable description of the parameter.
   * @param out Output vector to store the read values in.
   */
  void declare_and_get_array(
    const std::string & param_name,
    const std::string & description,
    std::vector<double> & out)
  {
    declare_parameter_if_not_declared(
      this, param_name, rclcpp::PARAMETER_DOUBLE_ARRAY,
      rcl_interfaces::msg::ParameterDescriptor().set__description(description));
    this->get_parameter(param_name, out);
  }

  std::string image_topic_;

  /// Target image dimensions after resize (-1 = no resize)
  int image_height_;
  int image_width_;

  /// Crop offsets (-1 = no crop)
  int offset_x_;
  int offset_y_;

  std::string frame_;

  bool enable_cam_info_{false};
  ip_camera_ros2::CalibrationParams calibration_;
  bool correct_cam_info_{false};

  rclcpp::CallbackGroup::SharedPtr cb_group_;
  rclcpp::TimerBase::SharedPtr timer_;

  cv_bridge::CvImage image_msg_;
  sensor_msgs::msg::CameraInfo cam_info_msg_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>> image_pub_;

  /// Only created when enable_cam_info_ && correct_cam_info_
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::CameraInfo>>
    cam_info_pub_;

  /**
   * @brief Timer callback that dequeues the latest frame and publishes it.
   */
  void capture_ipcam_image();

  /**
   * @brief Initialize the image message header and pixel encoding.
   * @return Pre-filled CvImage ready for stamping.
   */
  cv_bridge::CvImage initialize_image_msg();

  /**
   * @brief Declare and read all ROS2 node parameters.
   */
  void update_params();
};

#endif  // IP_CAMERA_ROS2__IP_CAMERA_ROS2_HPP_
