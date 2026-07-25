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

#include "ip_camera_ros2/frame_buffer.hpp"

#include <utility>

namespace ip_camera_ros2
{

FrameBuffer::FrameBuffer(size_t max_size)
: max_size_(max_size)
{
}

void FrameBuffer::push(cv::Mat frame)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (frames_.size() >= max_size_) {
    frames_.pop_front();  // O(1) removal of oldest frame
  }
  frames_.push_back(std::move(frame));
}

bool FrameBuffer::pop_latest(cv::Mat & out)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (frames_.empty()) {
    return false;
  }
  out = std::move(frames_.back());
  frames_.clear();
  return true;
}

}  // namespace ip_camera_ros2
