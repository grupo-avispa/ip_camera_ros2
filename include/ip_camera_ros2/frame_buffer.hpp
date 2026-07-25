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

#ifndef IP_CAMERA_ROS2__FRAME_BUFFER_HPP_
#define IP_CAMERA_ROS2__FRAME_BUFFER_HPP_

#include <deque>
#include <mutex>

#include "opencv2/opencv.hpp"

namespace ip_camera_ros2
{

/**
 * @brief Thread-safe hand-off point between the capture thread (producer) and the
 *        publishing timer callback (consumer).
 *
 * The consumer always reads the most recently pushed frame and discards any backlog,
 * so `max_size` only bounds the worst-case memory usage while the consumer is not
 * keeping up; it does not provide jitter smoothing.
 */
class FrameBuffer
{
public:
  /**
   * @brief Construct a FrameBuffer.
   * @param max_size Maximum number of frames kept before the oldest is dropped.
   */
  explicit FrameBuffer(size_t max_size);

  /**
   * @brief Push a frame from the producer thread.
   *
   * Drops the oldest buffered frame first if the buffer is already at `max_size`.
   *
   * @param frame Frame to push (moved in).
   */
  void push(cv::Mat frame);

  /**
   * @brief Pop the most recent frame and discard any older backlog.
   *
   * @param out Set to the most recent frame if one was available.
   * @return true if a frame was available and `out` was set, false if the buffer was empty.
   */
  bool pop_latest(cv::Mat & out);

private:
  mutable std::mutex mutex_;
  std::deque<cv::Mat> frames_;
  size_t max_size_;
};

}  // namespace ip_camera_ros2

#endif  // IP_CAMERA_ROS2__FRAME_BUFFER_HPP_
