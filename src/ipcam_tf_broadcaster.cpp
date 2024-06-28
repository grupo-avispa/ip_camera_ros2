// C++
#include <memory>
#include <string>

//ROS 2
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/string_utils.hpp"

using std::placeholders::_1;

class StaticFramePublisher : public rclcpp::Node
{
public: 
    StaticFramePublisher(): Node("static_ipcam_tf2_broadcaster"){
        tf_publisher_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        // Publish static transforms once at startup
        this->update_params();
        this->make_transforms();
    }

private:
    void make_transforms(){
        rclcpp::Time now = this->get_clock()->now();
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = now;
        t.header.frame_id = parent_frame_;
        t.child_frame_id = tf_;

        t.transform.translation.x = x_;
        t.transform.translation.y = y_;
        t.transform.translation.z = z_;
        tf2::Quaternion q;
        q.setRPY(roll_, pitch_,yaw_);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_publisher_->sendTransform(t);
    }
    void update_params(){
        // Static TF2 Transform parameters
        nav2_util::declare_parameter_if_not_declared(this, "x", rclcpp::ParameterValue(0.0), 
                                rcl_interfaces::msg::ParameterDescriptor()
                                .set__description("camera transform"));
        this->get_parameter("x", x_);
        RCLCPP_INFO(this->get_logger(), "The parameter x is set to: [%f]", x_);

        nav2_util::declare_parameter_if_not_declared(this, "y", rclcpp::ParameterValue(0.0),
                                rcl_interfaces::msg::ParameterDescriptor()
                                .set__description("camera transform"));
        this->get_parameter("y", y_);
        RCLCPP_INFO(this->get_logger(), "The parameter y is set to: [%f]", y_);

        nav2_util::declare_parameter_if_not_declared(this, "z", rclcpp::ParameterValue(0.0),
                                rcl_interfaces::msg::ParameterDescriptor()
                                .set__description("camera transform"));
        this->get_parameter("z", z_);
        RCLCPP_INFO(this->get_logger(), "The parameter z is set to: [%f]", z_);

        nav2_util::declare_parameter_if_not_declared(this, "yaw", rclcpp::ParameterValue(0.0), 
                                rcl_interfaces::msg::ParameterDescriptor()
                                .set__description("camera transform"));
        this->get_parameter("yaw", yaw_);
        RCLCPP_INFO(this->get_logger(), "The parameter yaw is set to: [%f]", yaw_);

        nav2_util::declare_parameter_if_not_declared(this, "pitch", rclcpp::ParameterValue(0.0), 
                                rcl_interfaces::msg::ParameterDescriptor()
                                .set__description("camera transform"));
        this->get_parameter("pitch", pitch_);
        RCLCPP_INFO(this->get_logger(), "The parameter pitch is set to: [%f]", pitch_);

        nav2_util::declare_parameter_if_not_declared(this, "roll", rclcpp::ParameterValue(0.0), 
                                rcl_interfaces::msg::ParameterDescriptor()
                                .set__description("camera transform"));
        this->get_parameter("roll", roll_);
        RCLCPP_INFO(this->get_logger(), "The parameter roll is set to: [%f]", roll_);

        nav2_util::declare_parameter_if_not_declared(this, "tf", rclcpp::ParameterValue("reolink"), 
                                rcl_interfaces::msg::ParameterDescriptor()
                                .set__description("camera transform"));
        this->get_parameter("tf", tf_);
        RCLCPP_INFO(this->get_logger(), "The parameter tf is set to: [%s]", tf_.c_str());

        nav2_util::declare_parameter_if_not_declared(this, "parent_frame", rclcpp::ParameterValue("map"), 
                                rcl_interfaces::msg::ParameterDescriptor()
                                .set__description("camera parent frame"));
        this->get_parameter("parent_frame", parent_frame_);
        RCLCPP_INFO(this->get_logger(), "The parameter parent_frame is set to: [%s]", parent_frame_.c_str());
    }
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_publisher_;
    // ROS Parameters for transform configuration
    float x_, y_, z_, roll_, pitch_, yaw_;
    std::string tf_, parent_frame_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
	rclcpp::executors::MultiThreadedExecutor exe;
	auto node = std::make_shared<StaticFramePublisher>();
	exe.add_node(node);
	exe.spin();
	rclcpp::shutdown();
	return 0;
}