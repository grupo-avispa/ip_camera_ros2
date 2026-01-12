// ROS 2
#include "sensor_msgs/image_encodings.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/string_utils.hpp"

#include "ip_camera_ros2/ip_camera_ros2.hpp"
#include <chrono>

IpCameraRos2::IpCameraRos2() : Node("ip_camera_ros2"){
    // Update ros2 params
    update_params();
    // Initialize publishers
    image_msg_ = initialize_image_msg();
    // Initialize frame buffer for producer-consumer pattern
    frame_buffer_.reserve(30);  // Pre-allocate space for efficiency
    // Initialize Callback Group
    cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    image_pub_ = image_transport::create_publisher(
        this,
        image_topic_);
    
    // Initialize Timer
    unsigned int frame_period_ms = static_cast<unsigned int>(1000 / frame_rate_);
    RCLCPP_INFO(this->get_logger(), "Frame period set to: %u ms", frame_period_ms);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(frame_period_ms),
      std::bind(&IpCameraRos2::capture_ipcam_image, this),
      cb_group_);
    
    if(enable_cam_info_){
        cam_info_msg_ = create_cam_info_msg(this->get_clock()->now());
        cam_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        cam_info_topic_,
        10);
    }
}

IpCameraRos2::~IpCameraRos2(){}

void IpCameraRos2::capture_ipcam_image(){
    // RCLCPP_INFO(this->get_logger(), "Image CAPTURED IN");
    // auto start = std::chrono::high_resolution_clock::now();
    
    cv::Mat local_frame;
    
    // Consumer: Get frame from buffer
    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        if (frame_buffer_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No frames available in buffer");
            return;
        }
        local_frame = frame_buffer_.back().clone();
        frame_buffer_.pop_back();
    }
    // Process image
    // Resize image
    if(image_width_ > 0 && image_height_ > 0){
        // Crop image
        if(offset_x_ > 0 && offset_y_ > 0){
            cv::Rect roi;
            roi.x = offset_x_;
            roi.y = offset_y_;
            roi.width = image_width_;
            roi.height = image_height_;
            local_frame = local_frame(roi);
        }
        // Only resize image
        else{
            cv::resize(local_frame, local_frame, cv::Size(image_width_, image_height_));
        }
    }
    // Create image msg
    rclcpp::Time stamp = this->get_clock()->now();
    image_msg_.header.stamp = stamp;
    image_msg_.image = local_frame;
    // Create camera info msg
    if(enable_cam_info_){
        // sensor_msgs::msg::CameraInfo cam_info_msg;
        cam_info_msg_.header.stamp = stamp;
        cam_info_pub_->publish(cam_info_msg_);
    }
    image_pub_.publish(*image_msg_.toImageMsg());
    
    // auto end = std::chrono::high_resolution_clock::now();
    // auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    // RCLCPP_INFO(this->get_logger(), "Capture elapsed time: %ld ms", elapsed.count());
}

cv_bridge::CvImage IpCameraRos2::initialize_image_msg(){
    cv_bridge::CvImage image_msg;
    image_msg.header.frame_id = frame_;
    image_msg.encoding = sensor_msgs::image_encodings::BGR8;
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

    nav2_util::declare_parameter_if_not_declared(this, "image_height", rclcpp::ParameterValue(-1), 
                            rcl_interfaces::msg::ParameterDescriptor()
                            .set__description("Depth image height"));
    this->get_parameter("image_height", image_height_);
    RCLCPP_INFO(this->get_logger(), "The parameter image_height is set to: [%d]", image_height_);

    nav2_util::declare_parameter_if_not_declared(this, "image_width", rclcpp::ParameterValue(-1), 
                            rcl_interfaces::msg::ParameterDescriptor()
                            .set__description("Depth image width"));
    this->get_parameter("image_width", image_width_);
    RCLCPP_INFO(this->get_logger(), "The parameter image_width is set to: [%d]", image_width_);

    nav2_util::declare_parameter_if_not_declared(this, "offset_x", rclcpp::ParameterValue(-1), 
                            rcl_interfaces::msg::ParameterDescriptor()
                            .set__description("Crop image offset X"));
    this->get_parameter("offset_x", offset_x_);
    RCLCPP_INFO(this->get_logger(), "The parameter offset_x is set to: [%d]", offset_x_);

    nav2_util::declare_parameter_if_not_declared(this, "offset_y", rclcpp::ParameterValue(-1), 
                            rcl_interfaces::msg::ParameterDescriptor()
                            .set__description("Crop image offset Y"));
    this->get_parameter("offset_y", offset_y_);
    RCLCPP_INFO(this->get_logger(), "The parameter offset_y is set to: [%d]", offset_y_);

    nav2_util::declare_parameter_if_not_declared(this, "url", rclcpp::ParameterValue("ipcam_url"), 
                            rcl_interfaces::msg::ParameterDescriptor()
                            .set__description("IP Cam url"));
    this->get_parameter("url", url_);

    nav2_util::declare_parameter_if_not_declared(this, "tf", rclcpp::ParameterValue("camera_optical_link"), 
                            rcl_interfaces::msg::ParameterDescriptor()
                            .set__description("camera frame"));
    this->get_parameter("tf", frame_);
    RCLCPP_INFO(this->get_logger(), "The parameter frame is set to: [%s]", frame_.c_str());

    nav2_util::declare_parameter_if_not_declared(this, "frame_rate", rclcpp::ParameterValue(30), 
                            rcl_interfaces::msg::ParameterDescriptor()
                            .set__description("frame rate for capturing images"));
    this->get_parameter("frame_rate", frame_rate_);
    RCLCPP_INFO(this->get_logger(), "The parameter frame_rate is set to: [%d]", frame_rate_);

    nav2_util::declare_parameter_if_not_declared(this, "buffer_size", rclcpp::ParameterValue(30), 
                            rcl_interfaces::msg::ParameterDescriptor()
                            .set__description("frame buffer size for the producer-consumer pattern"));
    this->get_parameter("buffer_size", buffer_size_);
    RCLCPP_INFO(this->get_logger(), "The parameter buffer_size is set to: [%zu]", buffer_size_);

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

/* RTSPCapturer Implementation */

RTSPCapturer::RTSPCapturer(const std::string& url, 
                            std::vector<cv::Mat>& frame_buffer, 
                            std::mutex& buffer_mutex,
                            size_t buffer_size)
    : url_(url), frames_(frame_buffer), buffer_mutex_(buffer_mutex), buffer_size_(buffer_size) {
}

RTSPCapturer::~RTSPCapturer() {
    cap_.release();
}

void RTSPCapturer::run() {
    cap_.open(url_, cv::CAP_FFMPEG);
    
    if (!cap_.isOpened()) {
        std::cerr << "Error: Could not open RTSP stream: " << url_ << std::endl;
        return;
    }
    
    std::cout << "RTSP Capturer started for: " << url_ << std::endl;
    
    cv::Mat frame;
    // Producer loop: capture frames continuously
    while (cap_.isOpened()) {
       // Grab a frame
        if (!cap_.grab()) {
            std::cerr << "Error: Cannot grab frame from RTSP stream." << std::endl;
            continue;
        }

        // Retrieve and process the frame
        cap_.retrieve(frame);
        
        if (frame.empty()) {
            std::cerr << "Warning: Empty frame captured" << std::endl;
            continue;
        }
        
        // Add frame to buffer (producer)
        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            // Keep buffer size limited (e.g., max 30 frames)
            if (frames_.size() >= buffer_size_) {
                frames_.erase(frames_.begin());
            }
            frames_.push_back(frame);
        }
        
        // Small delay to control capture rate
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}