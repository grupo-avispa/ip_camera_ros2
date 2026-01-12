#ifndef IP_CAMERA_ROS2__IPCAM_ROS2_
#define IP_CAMERA_ROS2__IPCAM_ROS2_

//C++
#include <string>
#include <vector>
#include <mutex>
#include "opencv2/opencv.hpp"

// ROS 2
#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "image_transport/image_transport.hpp"



class RTSPCapturer{
    public:
        /// Class constructor
        RTSPCapturer(const std::string& url, 
            std::vector<cv::Mat>& frame_buffer, 
            std::mutex& buffer_mutex,
            size_t buffer_size);
        
            /// Class destructor
        ~RTSPCapturer();
        
        /// Method to run the capturer loop
        void run();

    private:
        /// Video capture object
        cv::VideoCapture cap_;
        
        /// RTSP stream URL
        std::string url_;
        /// Shared frame buffer
        std::vector<cv::Mat>& frames_;

        /// Mutex for thread-safe access to frame buffer
        std::mutex& buffer_mutex_;

        /// Maximum buffer size
        size_t buffer_size_;
};

class IpCameraRos2 : public rclcpp::Node{
    public:
        /// Class constructor
        IpCameraRos2();

        /// Class destructor
        ~IpCameraRos2();

        /// Publisher for the ipcam image
        image_transport::Publisher image_pub_;

        /// Publisher for the camera info
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_pub_;

        /// Method called in the main loop for reading cap and publishing image
        void capture_ipcam_image();

        /// Public access to url for RTSPCapturer initialization
        std::string url_;

        /// Shared buffer for producer-consumer pattern
        std::vector<cv::Mat> frame_buffer_;

        /// Mutex for thread-safe access to frame buffer
        std::mutex buffer_mutex_;

        /// Frame rate for capturing images
        int frame_rate_;

        /// buffer size for the frame buffer
        size_t buffer_size_;

    private:
        /// Topic where ipcam image will be published
        std::string image_topic_;

        /// Topic where camera info will be published
        std::string cam_info_topic_;

        /// Final image height after resize (set to -1 if not used)
        int image_height_;

        /// Final image width after resize (set to -1 if not used)
        int image_width_;

        /// For image crop (set to -1 if not used)
        int offset_x_;

        /// For image crop (set to -1 if not used)
        int offset_y_;

        /// Camera selected frame
        std::string frame_;

        /// Enables camera info publishing
        bool enable_cam_info_;

        /// Calibration info to be published under camera info topic
        std::string distortion_model_;
        std::vector<double> k_, d_, r_, p_;
        bool correct_cam_info_ = false;

        // Callback group for timer
        rclcpp::CallbackGroup::SharedPtr cb_group_;

        // Timer for periodic image publishing
        rclcpp::TimerBase::SharedPtr timer_;

        // Mutex for thread safety between capturer and ROS2 node
        std::mutex mutex_;

        // create msgs as class member for efficiency
        sensor_msgs::msg::CameraInfo cam_info_msg_;
        cv_bridge::CvImage image_msg_;

        /// This method creates a image msg to be published
        cv_bridge::CvImage initialize_image_msg();

        /// This method creates a camera info msg to be published
        sensor_msgs::msg::CameraInfo create_cam_info_msg(rclcpp::Time stamp);

        /// This method update the ros2 params
        void update_params();
};
#endif // IP_CAMERA_ROS2__IPCAM_ROS2_