cmake_minimum_required(VERSION 3.8)
project(ip_camera_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ###############################################
# # Find  dependencies                         ##
# ###############################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)

# ##########
# # Build ##
# ##########
# # Specify additional locations of header files
# # Your package locations should be listed before other locations
include_directories(
    include
)

# Executable node
add_executable(${PROJECT_NAME}
    src/ip_camera_ros2.cpp
    src/rtsp_capturer.cpp
    src/frame_buffer.cpp
    src/image_ops.cpp
    src/camera_info_builder.cpp
    src/main.cpp
)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    sensor_msgs
    cv_bridge
    OpenCV
    image_transport
)

# ############
# # Install ##
# ############
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.hpp"

    # PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY urdf
    DESTINATION share/${PROJECT_NAME}/
)

# ############
# # Testing ##
# ############
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)

    # the following line skips the linter which checks for copyrights
    set(ament_cmake_copyright_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()

    add_subdirectory(test)
endif()

ament_package()
