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

#include "ip_camera_ros2/camera_info_builder.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

namespace ip_camera_ros2
{

bool is_calibration_valid(const CalibrationParams & calibration)
{
  return calibration.k.size() == 9 && calibration.r.size() == 9 && calibration.p.size() == 12;
}

namespace
{

/**
 * @brief Number of distortion coefficients a known distortion model uses.
 *
 * @param distortion_model Name of the distortion model.
 * @return The coefficient count, or std::nullopt for an unrecognized model.
 */
std::optional<size_t> expected_distortion_size(const std::string & distortion_model)
{
  if (distortion_model == "equidistant") {
    return 4;
  }
  if (distortion_model == "plumb_bob") {
    return 5;
  }
  if (distortion_model == "rational_polynomial") {
    return 8;
  }
  return std::nullopt;
}

}  // namespace

CalibrationCheck check_calibration(const CalibrationParams & calibration)
{
  CalibrationCheck check;

  if (calibration.k.size() != 9) {
    check.reason = "camera_matrix must have 9 elements, got " +
      std::to_string(calibration.k.size());
  } else if (calibration.r.size() != 9) {
    check.reason = "rectification_matrix must have 9 elements, got " +
      std::to_string(calibration.r.size());
  } else if (calibration.p.size() != 12) {
    check.reason = "projection_matrix must have 12 elements, got " +
      std::to_string(calibration.p.size());
  } else if (!(calibration.k[0] > 0.0) || !(calibration.k[4] > 0.0)) {
    check.reason = "camera_matrix has a non-positive focal length (fx = " +
      std::to_string(calibration.k[0]) + ", fy = " + std::to_string(calibration.k[4]) +
      "): the camera is not calibrated";
  } else if (calibration.k[8] != 1.0) {
    check.reason = "camera_matrix[8] must be 1, got " + std::to_string(calibration.k[8]);
  } else {
    const auto expected = expected_distortion_size(calibration.distortion_model);
    if (expected && calibration.d.size() != *expected) {
      check.reason = "distortion model '" + calibration.distortion_model + "' needs " +
        std::to_string(*expected) + " distortion coefficients, got " +
        std::to_string(calibration.d.size());
    } else if (!expected) {
      check.warning = "unrecognized distortion model '" + calibration.distortion_model +
        "': its distortion coefficients were not checked";
    }
  }

  check.valid = check.reason.empty();
  return check;
}

sensor_msgs::msg::CameraInfo build_camera_info(
  const CalibrationParams & calibration,
  const std::string & frame_id,
  rclcpp::Time stamp)
{
  sensor_msgs::msg::CameraInfo cam_info_msg;
  cam_info_msg.header.frame_id = frame_id;
  cam_info_msg.header.stamp = stamp;
  cam_info_msg.distortion_model = calibration.distortion_model;
  std::copy_n(
    calibration.k.begin(), std::min(calibration.k.size(), cam_info_msg.k.size()),
    cam_info_msg.k.begin());
  std::copy_n(
    calibration.p.begin(), std::min(calibration.p.size(), cam_info_msg.p.size()),
    cam_info_msg.p.begin());
  std::copy_n(
    calibration.r.begin(), std::min(calibration.r.size(), cam_info_msg.r.size()),
    cam_info_msg.r.begin());
  cam_info_msg.d = calibration.d;
  return cam_info_msg;
}

}  // namespace ip_camera_ros2
