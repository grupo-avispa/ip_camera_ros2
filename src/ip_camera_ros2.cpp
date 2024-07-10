// ROS 2
#include "sensor_msgs/image_encodings.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/string_utils.hpp"

#include "ip_camera_ros2/ip_camera_ros2.hpp"

IpCameraRos2::IpCameraRos2() : Node("ip_camera_ros2"){
    // Update ros2 params
    update_params();

    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    image_topic_,
    rclcpp::SensorDataQoS());
    if(enable_cam_info_){
        cam_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        cam_info_topic_,
        10);
    }

    // Initialize timers
    // Image publishing timer
    unsigned int publish_period = 1000/frame_rate_;
    image_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(publish_period), 
        std::bind(&IpCameraRos2::publish_ipcam_image, 
        this));
    RCLCPP_INFO(this->get_logger(), "Publishing image every [%d] ms", publish_period);

    // Open IP Cam stream
    cap_.open(url_);
    cap_.set(cv::CAP_PROP_FPS, frame_rate_);
}

IpCameraRos2::~IpCameraRos2(){
    cap_.release();
}

void IpCameraRos2::publish_ipcam_image(){
    // Check if camera parameters were set correctly
    if(!correct_cam_info_ && enable_cam_info_){
        RCLCPP_ERROR(this->get_logger(), 
            "Camera info parameters has some errors, check it o disable camera info publishing");
        return;
    }
    // Check video capture is corret
    if (!cap_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Could not open IP Cam at: [%s]", url_.c_str());
        RCLCPP_INFO(this->get_logger(), "Trying to open again Video Capture");
        // Retry to Open IP Cam stream
        cap_.open(url_);
        cap_.set(cv::CAP_PROP_FPS, frame_rate_);
        RCLCPP_INFO(this->get_logger(), "Successfully opened Video Capture");
        return;
    }
    cv::Mat cap_frame;
    cap_.read(cap_frame);
    // Check video frame is read correctly
    if (cap_frame.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Could not capture the frame, closing Video Capture");
        // Reset capture
        cap_.release();
        RCLCPP_INFO(this->get_logger(), "Video Capture closed");
        return;
    }
    // Resize image
    if(image_width_ > 0 && image_height_ > 0){
        // Crop image
        if(offset_x_ > 0 && offset_y_ > 0){
            cv::Rect roi;
            roi.x = offset_x_;
            roi.y = offset_y_;
            roi.width = image_width_;
            roi.height = image_height_;
            cap_frame = cap_frame(roi);
        }
        // Only resize image
        else{
            cv::resize(cap_frame, cap_frame, cv::Size(image_width_, image_height_));
        }
    }
    // Create image msg
    rclcpp::Time stamp = this->get_clock()->now();
    cv_bridge::CvImage image_msg;
    image_msg = create_image_msg(stamp, cap_frame);
    // Create camera info msg
    if(enable_cam_info_){
        sensor_msgs::msg::CameraInfo cam_info_msg;
        cam_info_msg = create_cam_info_msg(stamp);
        cam_info_pub_->publish(cam_info_msg);
    }
    image_pub_->publish(*image_msg.toImageMsg());
}

cv_bridge::CvImage IpCameraRos2::create_image_msg(rclcpp::Time stamp, cv::Mat &image){
    cv_bridge::CvImage image_msg;
    image_msg.header.stamp = stamp;
    image_msg.header.frame_id = frame_;
    image_msg.encoding = sensor_msgs::image_encodings::BGR8;
    image_msg.image = image;
    return image_msg;
}

sensor_msgs::msg::CameraInfo IpCameraRos2::create_cam_info_msg(rclcpp::Time stamp){
    sensor_msgs::msg::CameraInfo cam_info_msg;
    cam_info_msg.header.frame_id = frame_;
    cam_info_msg.header.stamp = stamp;
    cam_info_msg.height = image_height_;
    cam_info_msg.width = image_width_;
    cam_info_msg.distortion_model = distortion_model_;
    std::copy(k_.begin(), k_.end(), cam_info_msg.k.begin());
    std::copy(p_.begin(), p_.end(), cam_info_msg.p.begin());
    std::copy(r_.begin(), r_.end(), cam_info_msg.r.begin());
    cam_info_msg.d = d_;
    return cam_info_msg;
}

void IpCameraRos2::update_params(){

    // SDK parameters
    nav2_util::declare_parameter_if_not_declared(this, "image_topic", rclcpp::ParameterValue("/image"), 
                            rcl_interfaces::msg::ParameterDescriptor()
                            .set__description("Topic of the ip camera image"));
    this->get_parameter("image_topic", image_topic_);
    RCLCPP_INFO(this->get_logger(), "The parameter image_topic_ is set to: [%s]", image_topic_.c_str());

    nav2_util::declare_parameter_if_not_declared(this, "cam_info_topic", rclcpp::ParameterValue("/camera_info"), 
                            rcl_interfaces::msg::ParameterDescriptor()
                            .set__description("Topic of the ip camera image"));
    this->get_parameter("cam_info_topic", cam_info_topic_);
    RCLCPP_INFO(this->get_logger(), "The parameter cam_info_topic is set to: [%s]", cam_info_topic_.c_str());

    nav2_util::declare_parameter_if_not_declared(this, "image_height", rclcpp::ParameterValue(320), 
                            rcl_interfaces::msg::ParameterDescriptor()
                            .set__description("Depth image height"));
    this->get_parameter("image_height", image_height_);
    RCLCPP_INFO(this->get_logger(), "The parameter image_height is set to: [%d]", image_height_);

    nav2_util::declare_parameter_if_not_declared(this, "image_width", rclcpp::ParameterValue(900), 
                            rcl_interfaces::msg::ParameterDescriptor()
                            .set__description("Depth image width"));
    this->get_parameter("image_width", image_width_);
    RCLCPP_INFO(this->get_logger(), "The parameter image_width is set to: [%d]", image_width_);

    nav2_util::declare_parameter_if_not_declared(this, "offset_x", rclcpp::ParameterValue(100), 
                            rcl_interfaces::msg::ParameterDescriptor()
                            .set__description("Crop image offset X"));
    this->get_parameter("offset_x", offset_x_);
    RCLCPP_INFO(this->get_logger(), "The parameter offset_x is set to: [%d]", offset_x_);

    nav2_util::declare_parameter_if_not_declared(this, "offset_y", rclcpp::ParameterValue(100), 
                            rcl_interfaces::msg::ParameterDescriptor()
                            .set__description("Crop image offset Y"));
    this->get_parameter("offset_y", offset_y_);
    RCLCPP_INFO(this->get_logger(), "The parameter offset_y is set to: [%d]", offset_y_);

    nav2_util::declare_parameter_if_not_declared(this, "url", rclcpp::ParameterValue("ipcam_url"), 
                            rcl_interfaces::msg::ParameterDescriptor()
                            .set__description("IP Cam url"));
    this->get_parameter("url", url_);

    nav2_util::declare_parameter_if_not_declared(this, "tf", rclcpp::ParameterValue("reolink_optical_link"), 
                            rcl_interfaces::msg::ParameterDescriptor()
                            .set__description("camera frame"));
    this->get_parameter("tf", frame_);
    RCLCPP_INFO(this->get_logger(), "The parameter frame is set to: [%s]", frame_.c_str());

    nav2_util::declare_parameter_if_not_declared(this, "frame_rate", rclcpp::ParameterValue(30), 
                            rcl_interfaces::msg::ParameterDescriptor()
                            .set__description("Image publishing frame rate"));
    this->get_parameter("frame_rate", frame_rate_);
    RCLCPP_INFO(this->get_logger(), "The parameter frame_rate is set to: [%d]", frame_rate_);

    // Calibration parameters
    nav2_util::declare_parameter_if_not_declared(this, "enable_cam_info", rclcpp::ParameterValue(false), 
                            rcl_interfaces::msg::ParameterDescriptor()
                            .set__description("enable camera info publishing"));
    this->get_parameter("enable_cam_info", enable_cam_info_);
    if(enable_cam_info_)
    {
        RCLCPP_INFO(this->get_logger(), "The parameter enable_cam_info is set to: [true]");
        nav2_util::declare_parameter_if_not_declared(this, "distortion_model", rclcpp::ParameterValue("distortion_model"), 
                                rcl_interfaces::msg::ParameterDescriptor()
                                .set__description("camera calibration parameter"));
        this->get_parameter("distortion_model", distortion_model_);
        RCLCPP_INFO(this->get_logger(), "The parameter distortion_model is set to: [%s]", distortion_model_.c_str());

        nav2_util::declare_parameter_if_not_declared(this, "camera_matrix", 
                                rclcpp::PARAMETER_DOUBLE_ARRAY, 
                                rcl_interfaces::msg::ParameterDescriptor()
                                .set__description("camera calibration parameter"));
        this->get_parameter("camera_matrix", k_);

        nav2_util::declare_parameter_if_not_declared(this, "distortion_coefficients", 
                        rclcpp::PARAMETER_DOUBLE_ARRAY, 
                                rcl_interfaces::msg::ParameterDescriptor()
                                .set__description("camera calibration parameter"));
        this->get_parameter("distortion_coefficients", d_);

        nav2_util::declare_parameter_if_not_declared(this, "rectification_matrix", 
                        rclcpp::PARAMETER_DOUBLE_ARRAY, 
                                rcl_interfaces::msg::ParameterDescriptor()
                                .set__description("camera calibration parameter"));
        this->get_parameter("rectification_matrix", r_);

        nav2_util::declare_parameter_if_not_declared(this, "projection_matrix", 
                        rclcpp::PARAMETER_DOUBLE_ARRAY, 
                                rcl_interfaces::msg::ParameterDescriptor()
                                .set__description("camera calibration parameter"));
        this->get_parameter("projection_matrix", p_);

        if(k_.size() != 9 || r_.size() != 9 || p_.size() != 12){
            RCLCPP_INFO(this->get_logger(), "Setting calibration camera values went wrong");
        }else{
            RCLCPP_INFO(this->get_logger(), "Correct Calibration camera values");
            correct_cam_info_ = true;
        }
    }else{
        RCLCPP_INFO(this->get_logger(), "The parameter enable_cam_info is set to: [false]");
    }
}