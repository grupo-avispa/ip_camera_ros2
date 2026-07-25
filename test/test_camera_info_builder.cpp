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
#include "ip_camera_ros2/camera_info_builder.hpp"

namespace
{

ip_camera_ros2::CalibrationParams make_valid_calibration()
{
  ip_camera_ros2::CalibrationParams calibration;
  calibration.distortion_model = "plumb_bob";
  calibration.k = {1, 2, 3, 4, 5, 6, 7, 8, 9};
  calibration.d = {0.1, 0.2, 0.3, 0.4};
  calibration.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  calibration.p = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
  return calibration;
}

}  // namespace

TEST(IsCalibrationValid, AcceptsCorrectSizes) {
  EXPECT_TRUE(ip_camera_ros2::is_calibration_valid(make_valid_calibration()));
}

TEST(IsCalibrationValid, RejectsWrongCameraMatrixSize) {
  auto calibration = make_valid_calibration();
  calibration.k.push_back(10.0);
  EXPECT_FALSE(ip_camera_ros2::is_calibration_valid(calibration));
}

TEST(IsCalibrationValid, RejectsWrongRectificationMatrixSize) {
  auto calibration = make_valid_calibration();
  calibration.r.pop_back();
  EXPECT_FALSE(ip_camera_ros2::is_calibration_valid(calibration));
}

TEST(IsCalibrationValid, RejectsWrongProjectionMatrixSize) {
  auto calibration = make_valid_calibration();
  calibration.p.pop_back();
  EXPECT_FALSE(ip_camera_ros2::is_calibration_valid(calibration));
}

TEST(IsCalibrationValid, DistortionCoefficientsSizeIsNotConstrained) {
  auto calibration = make_valid_calibration();
  calibration.d.clear();
  EXPECT_TRUE(ip_camera_ros2::is_calibration_valid(calibration));
}

TEST(BuildCameraInfo, PopulatesHeaderAndCalibrationFields) {
  const auto calibration = make_valid_calibration();
  const rclcpp::Time stamp(123, 456, RCL_ROS_TIME);

  const auto msg = ip_camera_ros2::build_camera_info(calibration, "camera_optical_link", stamp);

  EXPECT_EQ(msg.header.frame_id, "camera_optical_link");
  EXPECT_EQ(msg.header.stamp.sec, 123);
  EXPECT_EQ(msg.header.stamp.nanosec, 456u);
  EXPECT_EQ(msg.distortion_model, "plumb_bob");
  for (size_t i = 0; i < calibration.k.size(); ++i) {
    EXPECT_DOUBLE_EQ(msg.k[i], calibration.k[i]);
  }
  for (size_t i = 0; i < calibration.r.size(); ++i) {
    EXPECT_DOUBLE_EQ(msg.r[i], calibration.r[i]);
  }
  for (size_t i = 0; i < calibration.p.size(); ++i) {
    EXPECT_DOUBLE_EQ(msg.p[i], calibration.p[i]);
  }
  ASSERT_EQ(msg.d.size(), calibration.d.size());
  for (size_t i = 0; i < calibration.d.size(); ++i) {
    EXPECT_DOUBLE_EQ(msg.d[i], calibration.d[i]);
  }
}

TEST(BuildCameraInfo, OversizedVectorsAreBoundedNotOverflowed) {
  // Regression test for [C3]: oversized calibration vectors must never write past the
  // fixed-size std::array fields of sensor_msgs/CameraInfo.
  ip_camera_ros2::CalibrationParams calibration = make_valid_calibration();
  calibration.k.insert(calibration.k.end(), {10.0, 11.0});
  calibration.r.insert(calibration.r.end(), {10.0, 11.0});
  calibration.p.insert(calibration.p.end(), {13.0, 14.0});

  const rclcpp::Time stamp(0, 0, RCL_ROS_TIME);
  const auto msg = ip_camera_ros2::build_camera_info(calibration, "frame", stamp);

  EXPECT_EQ(msg.k.size(), 9u);
  EXPECT_EQ(msg.r.size(), 9u);
  EXPECT_EQ(msg.p.size(), 12u);
  for (size_t i = 0; i < msg.k.size(); ++i) {
    EXPECT_DOUBLE_EQ(msg.k[i], calibration.k[i]);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
