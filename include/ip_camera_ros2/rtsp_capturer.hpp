#ifndef IP_CAMERA_ROS2__RTSP_CAPTURER_HPP_
#define IP_CAMERA_ROS2__RTSP_CAPTURER_HPP_

// C++
#include <atomic>
#include <deque>
#include <mutex>
#include <string>

#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @brief Captures frames from an RTSP stream in a dedicated producer thread.
 *
 * Implements automatic reconnection with exponential backoff when the stream
 * fails to open or drops unexpectedly during operation.
 */
class RTSPCapturer
{
public:
  /**
   * @brief Construct a new RTSPCapturer.
   *
   * @param url          RTSP stream URL.
   * @param frame_buffer Shared frame buffer (owned by IpCameraRos2).
   * @param buffer_mutex Mutex protecting the shared buffer.
   * @param buffer_size  Maximum number of frames to keep in the buffer.
   * @param logger       ROS2 logger for diagnostics.
   */
  RTSPCapturer(
    const std::string & url,
    std::deque<cv::Mat> & frame_buffer,
    std::mutex & buffer_mutex,
    size_t buffer_size,
    rclcpp::Logger logger);

  /**
   * @brief Destroy the RTSPCapturer and release capture resources.
   */
  ~RTSPCapturer();

  /**
   * @brief Run the capture loop (blocks until stop() is called).
   *
   * Connects to the RTSP stream and continuously reads frames into the shared
   * buffer. Reconnects automatically with exponential backoff on any failure.
   */
  void run();

  /**
   * @brief Signal the capture loop to stop and release the stream.
   */
  void stop();

private:
  /**
   * @brief Open the RTSP stream with reliable transport settings.
   *
   * Sets TCP transport and connection timeouts via FFMPEG capture options
   * before calling cap_.open().
   *
   * @return true if the stream was opened successfully, false otherwise.
   */
  bool connect();

  std::string url_;
  std::deque<cv::Mat> & frames_;
  std::mutex & buffer_mutex_;
  size_t buffer_size_;
  rclcpp::Logger logger_;
  cv::VideoCapture cap_;
  std::atomic<bool> running_{false};

  /// Initial and maximum retry delay (seconds) for exponential backoff
  static constexpr int BASE_RETRY_DELAY_S = 1;
  static constexpr int MAX_RETRY_DELAY_S = 32;

  /// Consecutive grab failures before forcing a full reconnection
  static constexpr int MAX_GRAB_FAILURES = 5;
};

#endif  // IP_CAMERA_ROS2__RTSP_CAPTURER_HPP_
