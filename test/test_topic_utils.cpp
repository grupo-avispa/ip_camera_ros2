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

#include "gtest/gtest.h"
#include "ip_camera_ros2/topic_utils.hpp"

TEST(DeriveCameraInfoTopic, RootTopic) {
  EXPECT_EQ(ip_camera_ros2::derive_camera_info_topic("/image"), "/camera_info");
}

TEST(DeriveCameraInfoTopic, NamespacedTopic) {
  EXPECT_EQ(
    ip_camera_ros2::derive_camera_info_topic("/camera/image_raw"), "/camera/camera_info");
}

TEST(DeriveCameraInfoTopic, RelativeTopicWithoutSlash) {
  EXPECT_EQ(ip_camera_ros2::derive_camera_info_topic("image_raw"), "/camera_info");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
