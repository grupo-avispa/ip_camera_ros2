#include "rclcpp/rclcpp.hpp"
#include "ip_camera_ros2/ip_camera_ros2.hpp"
#include <thread>

/* Main */

int main(int argc, char** argv){
	rclcpp::init(argc, argv);
	rclcpp::executors::SingleThreadedExecutor exe;
	auto node = std::make_shared<IpCameraRos2>();
	
	// Create RTSPCapturer with shared buffer and mutex
	auto cam_capturer = std::make_shared<RTSPCapturer>(
		node->url_,
		node->frame_buffer_,
		node->buffer_mutex_,
		node->buffer_size_
	);
	
	// Run capturer in separate thread
	std::thread capturer_thread([cam_capturer]() {
		cam_capturer->run();
	});
	
	exe.add_node(node);
	exe.spin();
	
	// Cleanup
	if (capturer_thread.joinable()) {
		capturer_thread.join();
	}
	
	rclcpp::shutdown();
	return 0;
}