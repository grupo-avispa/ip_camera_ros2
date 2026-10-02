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
#include "ip_camera_ros2/frame_buffer.hpp"

namespace
{

/// @brief 1x1 frame whose single pixel value identifies it in assertions, stamped with its value.
ip_camera_ros2::StampedFrame make_marker_frame(uchar value)
{
  return ip_camera_ros2::StampedFrame{
    cv::Mat(1, 1, CV_8UC1, cv::Scalar(value)), rclcpp::Time(value, 500, RCL_ROS_TIME)};
}

}  // namespace

TEST(FrameBuffer, PopLatestOnEmptyBufferReturnsFalse) {
  ip_camera_ros2::FrameBuffer buffer(2);
  ip_camera_ros2::StampedFrame out;
  EXPECT_FALSE(buffer.pop_latest(out));
}

TEST(FrameBuffer, PopLatestReturnsThePushedFrame) {
  ip_camera_ros2::FrameBuffer buffer(2);
  buffer.push(make_marker_frame(42));

  ip_camera_ros2::StampedFrame out;
  ASSERT_TRUE(buffer.pop_latest(out));
  EXPECT_EQ(out.image.at<uchar>(0, 0), 42);
}

TEST(FrameBuffer, PopLatestDrainsTheWholeBacklog) {
  ip_camera_ros2::FrameBuffer buffer(5);
  buffer.push(make_marker_frame(1));
  buffer.push(make_marker_frame(2));
  buffer.push(make_marker_frame(3));

  ip_camera_ros2::StampedFrame out;
  ASSERT_TRUE(buffer.pop_latest(out));
  EXPECT_EQ(out.image.at<uchar>(0, 0), 3);

  // The backlog (frames 1 and 2) must have been discarded, not just the most recent one.
  EXPECT_FALSE(buffer.pop_latest(out));
}

TEST(FrameBuffer, DropsOldestFrameOnceMaxSizeIsReached) {
  ip_camera_ros2::FrameBuffer buffer(1);
  buffer.push(make_marker_frame(1));
  buffer.push(make_marker_frame(2));

  ip_camera_ros2::StampedFrame out;
  ASSERT_TRUE(buffer.pop_latest(out));
  EXPECT_EQ(out.image.at<uchar>(0, 0), 2);
}

TEST(FrameBuffer, PopLatestKeepsTheStampOfTheMostRecentFrame) {
  ip_camera_ros2::FrameBuffer buffer(3);
  buffer.push(make_marker_frame(1));
  buffer.push(make_marker_frame(2));

  ip_camera_ros2::StampedFrame out;
  ASSERT_TRUE(buffer.pop_latest(out));
  EXPECT_EQ(out.stamp.seconds(), 2.0000005);
  EXPECT_EQ(out.stamp.get_clock_type(), RCL_ROS_TIME);
}

TEST(StampedFrame, DefaultStampIsZero) {
  const ip_camera_ros2::StampedFrame frame;
  EXPECT_TRUE(frame.image.empty());
  EXPECT_EQ(frame.stamp.nanoseconds(), 0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
