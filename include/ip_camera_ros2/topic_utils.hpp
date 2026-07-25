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

#ifndef IP_CAMERA_ROS2__TOPIC_UTILS_HPP_
#define IP_CAMERA_ROS2__TOPIC_UTILS_HPP_

#include <string>

namespace ip_camera_ros2
{

/**
 * @brief Derive the standard CameraInfo sibling topic from a base image topic.
 *
 * Mirrors `image_transport::getCameraInfoTopic()`'s convention: replaces the last path
 * component of `image_topic` with `camera_info` (e.g. "/image" -> "/camera_info",
 * "camera/image_raw" -> "camera/camera_info"), which is what calibration/rectification
 * tools expect.
 *
 * @param image_topic Base image topic.
 * @return The sibling CameraInfo topic.
 */
std::string derive_camera_info_topic(const std::string & image_topic);

}  // namespace ip_camera_ros2

#endif  // IP_CAMERA_ROS2__TOPIC_UTILS_HPP_
