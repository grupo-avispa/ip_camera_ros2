#ifndef IP_CAMERA_ROS2__IPCAM_ROS2_
#define IP_CAMERA_ROS2__IPCAM_ROS2_

//C++
#include <string>
#include <vector>
#include "opencv2/opencv.hpp"

// ROS 2
#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

class IpCameraRos2 : public rclcpp::Node{
    public:
        /// Class constructor
        IpCameraRos2();

        /// Class destructor
        ~IpCameraRos2();

        /// Publisher for the ipcam image
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

        /// Publisher for the camera info
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_pub_;

        /// Timer for image publishing at a certain rate:
		rclcpp::TimerBase::SharedPtr image_timer_;

        /// Timer for camera info publishing at a certain rate:
		rclcpp::TimerBase::SharedPtr cam_info_timer_;

    private:
        /// Topic where ipcam image will be published
        std::string image_topic_;

        /// Topic where camera info will be published
        std::string cam_info_topic_;

        /// Final image height after resize (set to -1 if not used)
        int image_height_;

        /// Final image width after resize (set to -1 if not used)
        int image_width_;

        /// Camera selected frame
        std::string frame_;

        /// Image publishing frame rate in HZ
        unsigned int frame_rate_;

        /// Url for ipcam connection
        std::string url_;

        /// Video capture for the ip camera
        cv::VideoCapture cap_;

        /// Enables camera info publishing
        bool enable_cam_info_;

        /// Calibration info to be published under camera info topic
        std::string distortion_model_;
        std::vector<double> k_, d_, r_, p_;
        bool correct_cam_info_ = false;

        /// This method converts cv image to ros and publishes the msg
        void publish_ipcam_image();

        /// This method creates a image msg to be published
        cv_bridge::CvImage create_image_msg(rclcpp::Time stamp, cv::Mat &image);

        /// This method creates a camera info msg to be published
        sensor_msgs::msg::CameraInfo create_cam_info_msg(rclcpp::Time stamp);

        /// This method update the ros2 params
        void update_params();
};
#endif // IP_CAMERA_ROS2__IPCAM_ROS2_