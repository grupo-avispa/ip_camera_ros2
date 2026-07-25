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

// C++
#include <algorithm>
#include <cstdlib>
#include <thread>

#include "ip_camera_ros2/rtsp_capturer.hpp"

RTSPCapturer::RTSPCapturer(
  const std::string & url,
  std::deque<cv::Mat> & frame_buffer,
  std::mutex & buffer_mutex,
  size_t buffer_size,
  rclcpp::Logger logger)
: url_(url), frames_(frame_buffer), buffer_mutex_(buffer_mutex),
  buffer_size_(buffer_size), logger_(logger)
{
}

RTSPCapturer::~RTSPCapturer()
{
  stop();
}

bool RTSPCapturer::connect()
{
  cap_.release();

  // Force TCP transport and set a 5-second socket/connection timeout.
  // TCP is far more reliable than UDP, especially over WiFi or across NAT.
  // Format: "key;value|key;value"
  setenv("OPENCV_FFMPEG_CAPTURE_OPTIONS", "rtsp_transport;tcp|stimeout;5000000", 1);

  cap_.open(url_, cv::CAP_FFMPEG);

  if (!cap_.isOpened()) {
    return false;
  }

  // Keep the internal decoder buffer at 1 frame to minimise latency
  cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);
  return true;
}

int RTSPCapturer::next_retry_delay(int current_delay_s)
{
  return std::min(current_delay_s * 2, MAX_RETRY_DELAY_S);
}

void RTSPCapturer::stop()
{
  running_ = false;
  // Releasing the capture object unblocks any pending grab() call.
  // cv::VideoCapture is not thread-safe in general, but releasing while
  // grab() is in progress is safe in practice with the FFmpeg backend.
  cap_.release();
}

void RTSPCapturer::run()
{
  running_ = true;
  int retry_delay_s = BASE_RETRY_DELAY_S;
  int consecutive_failures = 0;

  while (running_) {
    // ---- Connection phase ------------------------------------------------
    if (!cap_.isOpened()) {
      RCLCPP_INFO(logger_, "Connecting to RTSP stream: %s", url_.c_str());

      if (!connect()) {
        RCLCPP_WARN(
          logger_, "Failed to open RTSP stream. Retrying in %d s...", retry_delay_s);

        // Interruptible sleep: honours stop() in ≤100 ms
        for (int i = 0; i < retry_delay_s * 10 && running_; ++i) {
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        retry_delay_s = next_retry_delay(retry_delay_s);
        continue;
      }

      RCLCPP_INFO(logger_, "Connected to RTSP stream: %s", url_.c_str());
      retry_delay_s = BASE_RETRY_DELAY_S;
      consecutive_failures = 0;
    }

    // ---- Capture phase ---------------------------------------------------
    if (!cap_.grab()) {
      if (!running_) {
        break;
      }

      if (++consecutive_failures >= MAX_GRAB_FAILURES) {
        RCLCPP_WARN(
          logger_,
          "Lost RTSP stream after %d consecutive failures. Reconnecting...",
          consecutive_failures);
        cap_.release();
        consecutive_failures = 0;
      } else {
        RCLCPP_DEBUG(
          logger_, "Grab failed (%d/%d)", consecutive_failures, MAX_GRAB_FAILURES);
        // Avoid a tight busy-wait loop while the stream is in a transient failure state.
        std::this_thread::sleep_for(std::chrono::milliseconds(GRAB_FAILURE_SLEEP_MS));
      }
      continue;
    }

    consecutive_failures = 0;

    cv::Mat frame;
    if (!cap_.retrieve(frame) || frame.empty()) {
      RCLCPP_WARN(logger_, "Retrieved empty frame, skipping");
      std::this_thread::sleep_for(std::chrono::milliseconds(GRAB_FAILURE_SLEEP_MS));
      continue;
    }

    // Push to shared buffer (producer side)
    {
      std::lock_guard<std::mutex> lock(buffer_mutex_);
      if (frames_.size() >= buffer_size_) {
        frames_.pop_front();  // O(1) removal of oldest frame
      }
      frames_.push_back(std::move(frame));
    }
  }

  cap_.release();
  RCLCPP_INFO(logger_, "RTSP capturer stopped.");
}
