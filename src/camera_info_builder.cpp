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

namespace ip_camera_ros2
{

bool is_calibration_valid(const CalibrationParams & calibration)
{
  return calibration.k.size() == 9 && calibration.r.size() == 9 && calibration.p.size() == 12;
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
