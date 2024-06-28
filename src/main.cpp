#include "rclcpp/rclcpp.hpp"
#include "ip_camera_ros2/ip_camera_ros2.hpp"

/* Main */
int main(int argc, char** argv){
	rclcpp::init(argc, argv);
	rclcpp::executors::MultiThreadedExecutor exe;
	auto node = std::make_shared<IpCameraRos2>();
	exe.add_node(node);
	exe.spin();
	rclcpp::shutdown();
	return 0;
}