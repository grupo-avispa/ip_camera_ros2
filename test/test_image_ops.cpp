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
#include "ip_camera_ros2/image_ops.hpp"

namespace
{

/// @brief Build a synthetic frame where pixel (row, col) encodes its own coordinates.
cv::Mat make_test_frame(int width, int height)
{
  cv::Mat frame(height, width, CV_8UC3);
  for (int row = 0; row < height; ++row) {
    for (int col = 0; col < width; ++col) {
      frame.at<cv::Vec3b>(row, col) = cv::Vec3b(
        static_cast<uchar>(col % 256), static_cast<uchar>(row % 256), 0);
    }
  }
  return frame;
}

}  // namespace

TEST(ClampRoiToFrame, FullyInsideIsUnchanged) {
  const cv::Rect requested(10, 10, 50, 50);
  const cv::Rect clamped = ip_camera_ros2::clamp_roi_to_frame(requested, cv::Size(100, 100));
  EXPECT_EQ(clamped, requested);
}

TEST(ClampRoiToFrame, PartiallyOutsideIsShrunk) {
  const cv::Rect requested(80, 80, 50, 50);
  const cv::Rect clamped = ip_camera_ros2::clamp_roi_to_frame(requested, cv::Size(100, 100));
  EXPECT_EQ(clamped, cv::Rect(80, 80, 20, 20));
}

TEST(ClampRoiToFrame, FullyOutsideIsEmpty) {
  const cv::Rect requested(200, 200, 50, 50);
  const cv::Rect clamped = ip_camera_ros2::clamp_roi_to_frame(requested, cv::Size(100, 100));
  EXPECT_EQ(clamped.width, 0);
  EXPECT_EQ(clamped.height, 0);
}

TEST(ApplyCropOrResize, NoTransformReturnsSameFrame) {
  const cv::Mat frame = make_test_frame(80, 60);
  const ip_camera_ros2::CropRegion region{-1, -1, -1, -1};

  const cv::Mat result = ip_camera_ros2::apply_crop_or_resize(frame, region);

  ASSERT_EQ(result.cols, frame.cols);
  ASSERT_EQ(result.rows, frame.rows);
  EXPECT_EQ(result.at<cv::Vec3b>(0, 0), frame.at<cv::Vec3b>(0, 0));
}

TEST(ApplyCropOrResize, ResizeWhenNoOffsetGiven) {
  const cv::Mat frame = make_test_frame(80, 60);
  const ip_camera_ros2::CropRegion region{40, 30, -1, -1};

  const cv::Mat result = ip_camera_ros2::apply_crop_or_resize(frame, region);

  EXPECT_EQ(result.cols, 40);
  EXPECT_EQ(result.rows, 30);
}

TEST(ApplyCropOrResize, CropWithinBoundsExtractsExpectedRegion) {
  const cv::Mat frame = make_test_frame(80, 60);
  const ip_camera_ros2::CropRegion region{20, 10, 5, 8};

  const cv::Mat result = ip_camera_ros2::apply_crop_or_resize(frame, region);

  ASSERT_EQ(result.cols, 20);
  ASSERT_EQ(result.rows, 10);
  // Top-left pixel of the crop must match (offset_x, offset_y) in the source frame.
  EXPECT_EQ(result.at<cv::Vec3b>(0, 0), frame.at<cv::Vec3b>(8, 5));
}

TEST(ApplyCropOrResize, CropClampedToFrameBoundsShrinksInsteadOfThrowing) {
  const cv::Mat frame = make_test_frame(80, 60);
  // Requested ROI extends 20 px past both the right and bottom edges.
  const ip_camera_ros2::CropRegion region{30, 20, 70, 50};

  const cv::Mat result = ip_camera_ros2::apply_crop_or_resize(frame, region);

  EXPECT_EQ(result.cols, 10);
  EXPECT_EQ(result.rows, 10);
}

TEST(ApplyCropOrResize, CropFullyOutsideFrameReturnsEmptyMat) {
  const cv::Mat frame = make_test_frame(80, 60);
  const ip_camera_ros2::CropRegion region{20, 20, 200, 200};

  const cv::Mat result = ip_camera_ros2::apply_crop_or_resize(frame, region);

  EXPECT_TRUE(result.empty());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
