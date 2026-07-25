ament_add_gtest(test_image_ops test_image_ops.cpp ${CMAKE_SOURCE_DIR}/src/image_ops.cpp)
ament_target_dependencies(test_image_ops OpenCV)

ament_add_gtest(
  test_camera_info_builder
  test_camera_info_builder.cpp
  ${CMAKE_SOURCE_DIR}/src/camera_info_builder.cpp
)
ament_target_dependencies(test_camera_info_builder rclcpp sensor_msgs)

ament_add_gtest(
  test_rtsp_capturer_backoff
  test_rtsp_capturer_backoff.cpp
  ${CMAKE_SOURCE_DIR}/src/rtsp_capturer.cpp
)
ament_target_dependencies(test_rtsp_capturer_backoff rclcpp OpenCV)
