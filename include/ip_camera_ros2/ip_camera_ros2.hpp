#ifndef IP_CAMERA_ROS2__IPCAM_ROS2_
#define IP_CAMERA_ROS2__IPCAM_ROS2_

// C++
#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include "opencv2/opencv.hpp"

// ROS 2
#include "cv_bridge/cv_bridge.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

// Forward declarations
class RTSPCapturer;

/**
 * @brief ROS2 node that publishes frames received from an IP / RTSP camera.
 *
 * Works in tandem with RTSPCapturer: the capturer fills a shared frame buffer
 * from a dedicated thread, and this node's timer callback dequeues the latest
 * frame and publishes it as sensor_msgs/Image (and optionally CameraInfo).
 */
class IpCameraRos2 : public rclcpp::Node
{
public:
  /**
   * @brief Construct and configure the IpCameraRos2 node.
   */
  IpCameraRos2();

  /**
   * @brief Destructor.
   */
  ~IpCameraRos2();

  // ---- Public fields accessed by main() to construct RTSPCapturer ----

  /// RTSP stream URL
  std::string url_;

  /// Shared frame buffer for the producer-consumer pattern
  std::deque<cv::Mat> frame_buffer_;

  /// Mutex protecting frame_buffer_
  std::mutex buffer_mutex_;

  /// Target publishing frame rate (fps)
  int frame_rate_;

  /// Maximum number of frames held in the buffer
  size_t buffer_size_;

private:
  /**
   * @brief Declares static ROS2 parameter and sets it to a given value if it was not already declared.
   *
   * @param node A node in which given parameter to be declared
   * @param param_name The name of parameter
   * @param default_value Parameter value to initialize with
   * @param parameter_descriptor Parameter descriptor (optional)
  */
  template<typename NodeT>
  void declare_parameter_if_not_declared(
    NodeT node,
    const std::string & param_name,
    const rclcpp::ParameterValue & default_value,
    const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor =
    rcl_interfaces::msg::ParameterDescriptor())
  {
    if (!node->has_parameter(param_name)) {
      node->declare_parameter(param_name, default_value, parameter_descriptor);
    }
  }

  /**
   * @brief Declares static ROS2 parameter with given type if it was not already declared.
   *
   * @param node A node in which given parameter to be declared
   * @param param_name The name of parameter
   * @param param_type The type of parameter
   * @param parameter_descriptor Parameter descriptor (optional)
  */
  template<typename NodeT>
  void declare_parameter_if_not_declared(
    NodeT node,
    const std::string & param_name,
    const rclcpp::ParameterType & param_type,
    const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor =
    rcl_interfaces::msg::ParameterDescriptor())
  {
    if (!node->has_parameter(param_name)) {
      node->declare_parameter(param_name, param_type, parameter_descriptor);
    }
  }

  std::string image_topic_;
  std::string cam_info_topic_;

  /// Target image dimensions after resize (-1 = no resize)
  int image_height_;
  int image_width_;

  /// Crop offsets (-1 = no crop)
  int offset_x_;
  int offset_y_;

  std::string frame_;

  bool enable_cam_info_{false};
  std::string distortion_model_;
  std::vector<double> k_, d_, r_, p_;
  bool correct_cam_info_{false};

  rclcpp::CallbackGroup::SharedPtr cb_group_;
  rclcpp::TimerBase::SharedPtr timer_;

  cv_bridge::CvImage image_msg_;
  sensor_msgs::msg::CameraInfo cam_info_msg_;

  image_transport::Publisher image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_pub_;

  /**
   * @brief Timer callback that dequeues the latest frame and publishes it.
   */
  void capture_ipcam_image();

  /**
   * @brief Initialize the image message header and pixel encoding.
   * @return Pre-filled CvImage ready for stamping.
   */
  cv_bridge::CvImage initialize_image_msg();

  /**
   * @brief Build a CameraInfo message from current calibration parameters.
   * @param stamp Timestamp for the message header.
   * @return Populated CameraInfo message.
   */
  sensor_msgs::msg::CameraInfo create_cam_info_msg(rclcpp::Time stamp);

  /**
   * @brief Declare and read all ROS2 node parameters.
   */
  void update_params();
};

#endif  // IP_CAMERA_ROS2__IPCAM_ROS2_
