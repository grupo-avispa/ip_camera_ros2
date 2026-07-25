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

#ifndef IP_CAMERA_ROS2__IMAGE_OPS_HPP_
#define IP_CAMERA_ROS2__IMAGE_OPS_HPP_

#include "opencv2/opencv.hpp"

namespace ip_camera_ros2
{

/**
 * @brief Requested crop/resize transform, as read from ROS2 parameters.
 *
 * A crop is applied when `width`/`height` and `offset_x`/`offset_y` are all >= 0.
 * A resize (no crop) is applied when only `width`/`height` are >= 0.
 * No transform is applied when `width` or `height` is negative.
 */
struct CropRegion
{
  int width{-1};
  int height{-1};
  int offset_x{-1};
  int offset_y{-1};
};

/**
 * @brief Intersect the requested crop rectangle with the actual frame bounds.
 *
 * `cv::Mat::operator()(cv::Rect)` throws `cv::Exception` when the ROI is not fully
 * contained in the frame, which is trivial to trigger with a misconfigured
 * offset/size or a camera that changes resolution. Clamping the ROI first avoids
 * that crash entirely.
 *
 * @param requested Configured crop rectangle (offset_x, offset_y, width, height).
 * @param frame_size Size of the actual frame the crop will be applied to.
 * @return The intersection of `requested` with the frame bounds. May be empty
 *         (`width == 0 || height == 0`) if the requested ROI falls entirely outside
 *         the frame.
 */
cv::Rect clamp_roi_to_frame(const cv::Rect & requested, const cv::Size & frame_size);

/**
 * @brief Apply the configured crop or resize transform to a frame.
 *
 * @param frame Input frame (not modified).
 * @param region Requested crop/resize configuration.
 * @return The transformed frame, or an empty `cv::Mat` if a crop was requested but the
 *         ROI falls entirely outside the frame bounds once clamped. When no transform
 *         is configured, `frame` is returned as-is (cheap header copy, no pixel data
 *         is duplicated).
 */
cv::Mat apply_crop_or_resize(const cv::Mat & frame, const CropRegion & region);

}  // namespace ip_camera_ros2

#endif  // IP_CAMERA_ROS2__IMAGE_OPS_HPP_
