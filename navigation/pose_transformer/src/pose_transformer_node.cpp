#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/buffer.h>
#include <message_filters/subscriber.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/create_timer_ros.h"


using std::placeholders::_1;

class PoseTransformerNode : public rclcpp::Node {
public:
    PoseTransformerNode()
    : Node("pose_transformer_node")
    {
        // Declare and acquire `target_frame` parameter
        target_frame_ = this->declare_parameter<std::string>("target_frame", "odom");

        std::chrono::duration<int> buffer_timeout(1);

        tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        // Create the timer interface before call to waitForTransform,
        // to avoid a tf2_ros::CreateTimerInterfaceException exception
        auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(),
        this->get_node_timers_interface());

        tf2_buffer_->setCreateTimerInterface(timer_interface);
        tf2_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

        // Adjust the QoS to match the publisher's QoS
        rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

        // Subscribe to the input PoseStamped topic
        pose_sub_.subscribe(this, "/aruco_board_pose_camera", qos.get_rmw_qos_profile());

        // Set up the MessageFilter to transform poses into the "odom" frame
        tf2_filter_ = std::make_shared<tf2_ros::MessageFilter<geometry_msgs::msg::PoseStamped>>(
            pose_sub_, *tf2_buffer_, "odom", 100, this->get_node_logging_interface(), this->get_node_clock_interface());

        tf2_filter_->registerCallback(std::bind(&PoseTransformerNode::poseCallback, this, _1));

        // Publisher for the transformed PoseStamped message
        transformed_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/aruco_board_pose_odom", 10);

        RCLCPP_INFO(this->get_logger(), "PoseTransformerNode has been started.");
    }

private:
    void poseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_msg)
    {
        // Attempt to transform the PoseStamped into the "odom" frame
        geometry_msgs::msg::PoseStamped transformed_pose;
        try {
            tf2_buffer_->transform(*pose_msg, transformed_pose, "odom", tf2::Duration(std::chrono::milliseconds(100)));
            transformed_pose_pub_->publish(transformed_pose);
            RCLCPP_INFO(this->get_logger(), "Pose transformed and published.");
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
        }
    }
    
    std::string target_frame_;
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

    // Subscriber and message filter for the input PoseStamped messages
    message_filters::Subscriber<geometry_msgs::msg::PoseStamped> pose_sub_;
    std::shared_ptr<tf2_ros::MessageFilter<geometry_msgs::msg::PoseStamped>> tf2_filter_;


    // Publisher for the transformed poses
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr transformed_pose_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseTransformerNode>());
    rclcpp::shutdown();
    return 0;
}
