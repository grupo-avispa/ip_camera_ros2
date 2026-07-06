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
#include <thread>

#include "ip_camera_ros2/ip_camera_ros2.hpp"
#include "ip_camera_ros2/rtsp_capturer.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;

  auto node = std::make_shared<IpCameraRos2>();
  exe.add_node(node);

  // Create the RTSP capturer sharing the node's buffer, mutex and logger
  auto cam_capturer = std::make_shared<RTSPCapturer>(
    node->url_,
    node->frame_buffer_,
    node->buffer_mutex_,
    node->buffer_size_,
    node->get_logger());

  // Run the capturer in a dedicated producer thread
  std::thread capturer_thread([cam_capturer]() {
      cam_capturer->run();
    });

  exe.spin();  // Blocks until rclcpp::shutdown() (e.g. Ctrl+C)

  // Signal the capturer to stop and wait for its thread to finish
  cam_capturer->stop();
  if (capturer_thread.joinable()) {
    capturer_thread.join();
  }

  rclcpp::shutdown();
  return 0;
}
