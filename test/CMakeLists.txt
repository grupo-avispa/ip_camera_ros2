# Tests link against the core library rather than recompiling its sources.
# Test for image operations
ament_add_gtest(test_image_ops test_image_ops.cpp)
target_link_libraries(test_image_ops ${library_name} ${OpenCV_LIBS})

# Test for topic name utilities
ament_add_gtest(test_topic_utils test_topic_utils.cpp)
target_link_libraries(test_topic_utils ${library_name})

# Test for the frame buffer
ament_add_gtest(test_frame_buffer test_frame_buffer.cpp)
target_link_libraries(test_frame_buffer ${library_name} ${OpenCV_LIBS})

# Test for the camera info builder
ament_add_gtest(test_camera_info_builder test_camera_info_builder.cpp)
target_link_libraries(test_camera_info_builder
  ${library_name}
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
)

# Test for the RTSP capturer reconnection backoff
ament_add_gtest(test_rtsp_capturer_backoff test_rtsp_capturer_backoff.cpp)
target_link_libraries(test_rtsp_capturer_backoff
  ${library_name}
  rclcpp::rclcpp
  ${OpenCV_LIBS}
)
