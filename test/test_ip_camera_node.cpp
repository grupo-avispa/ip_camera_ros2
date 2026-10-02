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

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/srv/set_camera_info.hpp"

#include "ip_camera_ros2/ip_camera_ros2.hpp"

using namespace std::chrono_literals;  // NOLINT

namespace
{

constexpr int kFrameWidth = 64;
constexpr int kFrameHeight = 48;

/// @brief Node whose frames are injected by the test instead of coming from a stream.
class TestableIpCamera : public IpCameraRos2
{
public:
  using IpCameraRos2::IpCameraRos2;

  /// @brief Put a frame in the buffer as if the capture thread had received it.
  void inject(const rclcpp::Time & stamp)
  {
    frame_buffer_->push(
      ip_camera_ros2::StampedFrame{
        cv::Mat(kFrameHeight, kFrameWidth, CV_8UC3, cv::Scalar(10, 20, 30)), stamp});
  }

protected:
  /// @brief Skip opening the RTSP stream.
  void start_capture() override {}
};

/// @brief Parameters of a valid fisheye calibration for a frame of the test size.
std::vector<rclcpp::Parameter> calibrated_parameters(bool enabled = true)
{
  return {
    {"frame_rate", 50},
    {"enable_cam_info", enabled},
    {"distortion_model", "equidistant"},
    {"camera_matrix", std::vector<double>{30.0, 0.0, 32.0, 0.0, 30.0, 24.0, 0.0, 0.0, 1.0}},
    {"distortion_coefficients", std::vector<double>{0.01, 0.0, 0.0, 0.0}},
    {"rectification_matrix", std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}},
    {"projection_matrix",
      std::vector<double>{30.0, 0.0, 32.0, 0.0, 0.0, 30.0, 24.0, 0.0, 0.0, 0.0, 1.0, 0.0}},
    {"calibration_width", kFrameWidth},
    {"calibration_height", kFrameHeight},
  };
}

}  // namespace

/**
 * @brief Runs the node in an executor and collects what it publishes.
 */
class IpCameraNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void start(const std::vector<rclcpp::Parameter> & parameters)
  {
    node_ =
      std::make_shared<TestableIpCamera>(rclcpp::NodeOptions().parameter_overrides(parameters));
    tester_ = std::make_shared<rclcpp::Node>("ip_camera_tester");
    image_sub_ = tester_->create_subscription<sensor_msgs::msg::Image>(
      "/image", 10, [this](sensor_msgs::msg::Image::SharedPtr msg) {images_.push_back(msg);});
    info_sub_ = tester_->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera_info", 10,
      [this](sensor_msgs::msg::CameraInfo::SharedPtr msg) {infos_.push_back(msg);});
    set_info_client_ = tester_->create_client<sensor_msgs::srv::SetCameraInfo>("/set_camera_info");
    executor_.add_node(node_->get_node_base_interface());
    executor_.add_node(tester_);
    ASSERT_EQ(
      node_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  }

  void TearDown() override
  {
    if (node_) {
      executor_.remove_node(node_->get_node_base_interface());
      executor_.remove_node(tester_);
    }
  }

  /// @brief Spin until the predicate holds or the timeout expires.
  template<typename Predicate>
  bool spin_until(Predicate predicate, std::chrono::milliseconds timeout = 5000ms)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some(20ms);
      if (predicate()) {
        return true;
      }
    }
    return predicate();
  }

  /// @brief Spin for a fixed time.
  void spin_for(std::chrono::milliseconds duration)
  {
    spin_until([] {return false;}, duration);
  }

  /// @brief Call set_camera_info with a calibration and wait for the answer.
  sensor_msgs::srv::SetCameraInfo::Response::SharedPtr call_set_camera_info(
    const sensor_msgs::msg::CameraInfo & info)
  {
    EXPECT_TRUE(set_info_client_->wait_for_service(5s));
    auto request = std::make_shared<sensor_msgs::srv::SetCameraInfo::Request>();
    request->camera_info = info;
    auto future = set_info_client_->async_send_request(request);
    EXPECT_EQ(executor_.spin_until_future_complete(future, 5s), rclcpp::FutureReturnCode::SUCCESS);
    return future.get();
  }

  std::shared_ptr<TestableIpCamera> node_;
  std::shared_ptr<rclcpp::Node> tester_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Client<sensor_msgs::srv::SetCameraInfo>::SharedPtr set_info_client_;
  std::vector<sensor_msgs::msg::Image::SharedPtr> images_;
  std::vector<sensor_msgs::msg::CameraInfo::SharedPtr> infos_;
};

/// @brief The uncalibrated template (K all zeros) must not produce a CameraInfo.
TEST_F(IpCameraNodeTest, ZeroCameraMatrixPublishesNoCameraInfo)
{
  auto parameters = calibrated_parameters();
  parameters.emplace_back(
    "camera_matrix", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  start(parameters);

  node_->inject(tester_->now());
  ASSERT_TRUE(spin_until([this] {return !images_.empty();}));
  spin_for(200ms);
  EXPECT_TRUE(infos_.empty());
}

/// @brief A frame of another size than the calibrated one must not get a CameraInfo.
TEST_F(IpCameraNodeTest, CalibrationSizeMismatchPublishesNoCameraInfo)
{
  auto parameters = calibrated_parameters();
  parameters.emplace_back("calibration_width", kFrameWidth * 2);
  start(parameters);

  node_->inject(tester_->now());
  ASSERT_TRUE(spin_until([this] {return !images_.empty();}));
  spin_for(200ms);
  EXPECT_TRUE(infos_.empty());
}

/// @brief A valid calibration is published with the size of the frame.
TEST_F(IpCameraNodeTest, ValidCalibrationPublishesCameraInfo)
{
  start(calibrated_parameters());

  node_->inject(tester_->now());
  ASSERT_TRUE(spin_until([this] {return !infos_.empty();}));
  EXPECT_EQ(infos_.back()->distortion_model, "equidistant");
  EXPECT_EQ(infos_.back()->width, static_cast<uint32_t>(kFrameWidth));
  EXPECT_EQ(infos_.back()->height, static_cast<uint32_t>(kFrameHeight));
  EXPECT_EQ(infos_.back()->d.size(), 4u);
}

/// @brief With the default parameters nothing but the image is published.
TEST_F(IpCameraNodeTest, DefaultParametersPublishNoCameraInfo)
{
  start({{"frame_rate", 50}});

  node_->inject(tester_->now());
  ASSERT_TRUE(spin_until([this] {return !images_.empty();}));
  spin_for(200ms);
  EXPECT_TRUE(infos_.empty());
}

/// @brief Image and camera info carry the capture stamp minus stamp_offset.
TEST_F(IpCameraNodeTest, StampIsTheCaptureTimeMinusTheOffset)
{
  auto parameters = calibrated_parameters();
  parameters.emplace_back("stamp_offset", 0.25);
  start(parameters);

  const rclcpp::Time capture(100, 0, RCL_ROS_TIME);
  node_->inject(capture);
  ASSERT_TRUE(spin_until([this] {return !images_.empty() && !infos_.empty();}));

  const rclcpp::Time expected(99, 750000000, RCL_ROS_TIME);
  EXPECT_EQ(rclcpp::Time(images_.back()->header.stamp), expected);
  EXPECT_EQ(rclcpp::Time(infos_.back()->header.stamp), expected);
}

/// @brief A capture is published once even if it is injected again.
TEST_F(IpCameraNodeTest, RepeatedFrameIsPublishedOnce)
{
  start(calibrated_parameters());

  const rclcpp::Time capture(200, 0, RCL_ROS_TIME);
  node_->inject(capture);
  ASSERT_TRUE(spin_until([this] {return !images_.empty();}));
  node_->inject(capture);
  spin_for(300ms);
  EXPECT_EQ(images_.size(), 1u);

  node_->inject(rclcpp::Time(201, 0, RCL_ROS_TIME));
  EXPECT_TRUE(spin_until([this] {return images_.size() == 2u;}));
}

namespace
{

sensor_msgs::msg::CameraInfo make_calibration_message()
{
  sensor_msgs::msg::CameraInfo info;
  info.width = kFrameWidth;
  info.height = kFrameHeight;
  info.distortion_model = "equidistant";
  info.k = {31.0, 0.0, 33.0, 0.0, 31.5, 25.0, 0.0, 0.0, 1.0};
  info.d = {0.02, -0.01, 0.003, 0.0};
  info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  info.p = {31.0, 0.0, 33.0, 0.0, 0.0, 31.5, 25.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  return info;
}

}  // namespace

/// @brief A valid calibration sent to the service becomes node parameters and is published.
TEST_F(IpCameraNodeTest, SetCameraInfoAppliesTheCalibrationAsParameters)
{
  start({{"frame_rate", 50}});

  const auto response = call_set_camera_info(make_calibration_message());
  ASSERT_TRUE(response->success) << response->status_message;

  EXPECT_TRUE(node_->get_parameter("enable_cam_info").as_bool());
  EXPECT_EQ(node_->get_parameter("distortion_model").as_string(), "equidistant");
  EXPECT_EQ(node_->get_parameter("calibration_width").as_int(), kFrameWidth);
  EXPECT_EQ(node_->get_parameter("calibration_height").as_int(), kFrameHeight);
  const auto camera_matrix = node_->get_parameter("camera_matrix").as_double_array();
  ASSERT_EQ(camera_matrix.size(), 9u);
  EXPECT_DOUBLE_EQ(camera_matrix[0], 31.0);
  EXPECT_DOUBLE_EQ(node_->get_parameter("distortion_coefficients").as_double_array()[1], -0.01);

  node_->inject(tester_->now());
  ASSERT_TRUE(spin_until([this] {return !infos_.empty();}));
  EXPECT_DOUBLE_EQ(infos_.back()->k[0], 31.0);
  EXPECT_DOUBLE_EQ(infos_.back()->k[4], 31.5);
  EXPECT_DOUBLE_EQ(infos_.back()->d[1], -0.01);
}

/// @brief An invalid calibration is refused with a reason and changes nothing.
TEST_F(IpCameraNodeTest, SetCameraInfoRejectsAnInvalidCalibration)
{
  start({{"frame_rate", 50}});

  auto info = make_calibration_message();
  info.k = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const auto response = call_set_camera_info(info);
  EXPECT_FALSE(response->success);
  EXPECT_NE(response->status_message.find("focal length"), std::string::npos);
  EXPECT_FALSE(node_->get_parameter("enable_cam_info").as_bool());

  node_->inject(tester_->now());
  ASSERT_TRUE(spin_until([this] {return !images_.empty();}));
  spin_for(200ms);
  EXPECT_TRUE(infos_.empty());
}

/// @brief Calibration parameters can be changed at runtime, and a stale size stops publishing.
TEST_F(IpCameraNodeTest, ChangingTheCalibrationSizeAtRuntimeStopsCameraInfo)
{
  start(calibrated_parameters());

  node_->inject(rclcpp::Time(300, 0, RCL_ROS_TIME));
  ASSERT_TRUE(spin_until([this] {return !infos_.empty();}));
  const size_t published = infos_.size();

  ASSERT_TRUE(node_->set_parameter(rclcpp::Parameter("calibration_width", 1920)).successful);
  node_->inject(rclcpp::Time(301, 0, RCL_ROS_TIME));
  ASSERT_TRUE(spin_until([this] {return images_.size() == 2u;}));
  spin_for(200ms);
  EXPECT_EQ(infos_.size(), published);
}

/// @brief The lifecycle can be cycled and the node configured again.
TEST_F(IpCameraNodeTest, LifecycleCycles)
{
  start(calibrated_parameters());
  ASSERT_EQ(node_->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node_->cleanup().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  ASSERT_EQ(node_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
}
