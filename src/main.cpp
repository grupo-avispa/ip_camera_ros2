#include "rclcpp/rclcpp.hpp"
#include "ip_camera_ros2/ip_camera_ros2.hpp"
#include "ip_camera_ros2/rtsp_capturer.hpp"
#include <thread>

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
