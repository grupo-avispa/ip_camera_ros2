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

#include "ip_camera_ros2/topic_utils.hpp"

namespace ip_camera_ros2
{

std::string derive_camera_info_topic(const std::string & image_topic)
{
  const auto last_slash = image_topic.find_last_of('/');
  const std::string parent =
    (last_slash == std::string::npos) ? std::string() : image_topic.substr(0, last_slash);
  return parent + "/camera_info";
}

}  // namespace ip_camera_ros2
