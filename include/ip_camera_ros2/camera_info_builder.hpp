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

#ifndef IP_CAMERA_ROS2__CAMERA_INFO_BUILDER_HPP_
#define IP_CAMERA_ROS2__CAMERA_INFO_BUILDER_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace ip_camera_ros2
{

/// @brief Camera calibration parameters, as read from ROS2 parameters.
struct CalibrationParams
{
  std::string distortion_model;

  /// 3x3 intrinsic camera matrix, row-major (9 elements when valid)
  std::vector<double> k;

  /// Distortion coefficients (size depends on distortion_model)
  std::vector<double> d;

  /// 3x3 rectification matrix, row-major (9 elements when valid)
  std::vector<double> r;

  /// 3x4 projection matrix, row-major (12 elements when valid)
  std::vector<double> p;
};

/**
 * @brief Check whether the calibration matrices have the sizes CameraInfo expects.
 *
 * @param calibration Calibration parameters to validate.
 * @return true if `k`/`r`/`p` match the fixed sizes of `sensor_msgs/CameraInfo`
 *         (9, 9 and 12 elements respectively), false otherwise.
 */
bool is_calibration_valid(const CalibrationParams & calibration);

/**
 * @brief Build a CameraInfo message from calibration parameters.
 *
 * `height`/`width` are intentionally left at their default (0): the caller is expected
 * to fill them in from the actual published frame dimensions, since those may not match
 * the configured resize/crop parameters (e.g. an unset or clamped crop ROI).
 *
 * @param calibration Calibration parameters. Only sizes bounded by the destination
 *        `std::array` are copied, even if `is_calibration_valid()` was not checked first.
 * @param frame_id Frame ID for the message header.
 * @param stamp Timestamp for the message header.
 * @return Populated CameraInfo message.
 */
sensor_msgs::msg::CameraInfo build_camera_info(
  const CalibrationParams & calibration,
  const std::string & frame_id,
  rclcpp::Time stamp);

}  // namespace ip_camera_ros2

#endif  // IP_CAMERA_ROS2__CAMERA_INFO_BUILDER_HPP_
