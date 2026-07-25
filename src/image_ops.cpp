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

#include "ip_camera_ros2/image_ops.hpp"

namespace ip_camera_ros2
{

cv::Rect clamp_roi_to_frame(const cv::Rect & requested, const cv::Size & frame_size)
{
  const cv::Rect frame_rect(0, 0, frame_size.width, frame_size.height);
  return requested & frame_rect;
}

cv::Mat apply_crop_or_resize(const cv::Mat & frame, const CropRegion & region)
{
  if (region.width <= 0 || region.height <= 0) {
    return frame;
  }

  if (region.offset_x >= 0 && region.offset_y >= 0) {
    const cv::Rect requested(region.offset_x, region.offset_y, region.width, region.height);
    const cv::Rect roi = clamp_roi_to_frame(requested, frame.size());
    if (roi.width == 0 || roi.height == 0) {
      return cv::Mat();
    }
    return frame(roi).clone();
  }

  cv::Mat resized;
  cv::resize(frame, resized, cv::Size(region.width, region.height));
  return resized;
}

}  // namespace ip_camera_ros2
