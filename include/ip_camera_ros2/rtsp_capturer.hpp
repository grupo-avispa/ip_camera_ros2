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

#ifndef IP_CAMERA_ROS2__RTSP_CAPTURER_HPP_
#define IP_CAMERA_ROS2__RTSP_CAPTURER_HPP_

// C++
#include <atomic>
#include <string>

#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"

#include "ip_camera_ros2/frame_buffer.hpp"

/**
 * @brief Captures frames from an RTSP stream in a dedicated producer thread.
 *
 * Implements automatic reconnection with exponential backoff when the stream
 * fails to open or drops unexpectedly during operation.
 */
class RTSPCapturer
{
public:
  /**
   * @brief Construct a new RTSPCapturer.
   *
   * @param url          RTSP stream URL.
   * @param frame_buffer Shared frame buffer (owned by IpCameraRos2).
   * @param logger       ROS2 logger for diagnostics.
   */
  RTSPCapturer(
    const std::string & url,
    ip_camera_ros2::FrameBuffer & frame_buffer,
    rclcpp::Logger logger);

  /**
   * @brief Destroy the RTSPCapturer and release capture resources.
   */
  ~RTSPCapturer();

  /**
   * @brief Run the capture loop (blocks until stop() is called).
   *
   * Connects to the RTSP stream and continuously reads frames into the shared
   * buffer. Reconnects automatically with exponential backoff on any failure.
   */
  void run();

  /**
   * @brief Signal the capture loop to stop and release the stream.
   */
  void stop();

  /**
   * @brief Compute the next exponential backoff delay, capped at MAX_RETRY_DELAY_S.
   *
   * Pure function exposed for unit testing; used internally by run() between
   * reconnection attempts.
   *
   * @param current_delay_s Current retry delay, in seconds.
   * @return The next delay: `min(current_delay_s * 2, MAX_RETRY_DELAY_S)`.
   */
  static int next_retry_delay(int current_delay_s);

private:
  /**
   * @brief Open the RTSP stream with reliable transport settings.
   *
   * Sets TCP transport and connection timeouts via FFMPEG capture options
   * before calling cap_.open().
   *
   * @return true if the stream was opened successfully, false otherwise.
   */
  bool connect();

  std::string url_;
  ip_camera_ros2::FrameBuffer & frame_buffer_;
  rclcpp::Logger logger_;
  cv::VideoCapture cap_;
  std::atomic<bool> running_{false};

  /// Initial and maximum retry delay (seconds) for exponential backoff
  static constexpr int BASE_RETRY_DELAY_S = 1;
  static constexpr int MAX_RETRY_DELAY_S = 32;

  /// Consecutive grab failures before forcing a full reconnection
  static constexpr int MAX_GRAB_FAILURES = 5;

  /// Pause applied between retries on a failed grab()/empty frame to avoid a busy-wait
  static constexpr int GRAB_FAILURE_SLEEP_MS = 10;
};

#endif  // IP_CAMERA_ROS2__RTSP_CAPTURER_HPP_
