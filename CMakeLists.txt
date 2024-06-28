cmake_minimum_required(VERSION 3.8)
project(ip_camera_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()


################################################
## Find  dependencies                         ##
################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_util REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
)

# Executable node
add_executable (${PROJECT_NAME} 
    src/ip_camera_ros2.cpp
    src/main.cpp
)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    nav2_util
    sensor_msgs
    cv_bridge
    OpenCV
)

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME})

#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.hpp"
    #   PATTERN ".svn" EXCLUDE
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

#############
## Testing ##
#############
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
