#ifndef IPDA_POSE_FILTERING_ROS_HPP
#define IPDA_POSE_FILTERING_ROS_HPP

#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace vortex::filtering {

class IPDAPoseFilteringNode : public rclcpp::Node {
   public:
    IPDAPoseFilteringNode(const rclcpp::NodeOptions& options);

    ~IPDAPoseFilteringNode() {};

   private:
    void setup_publishers_and_subscribers();

    void pose_callback(
        const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);

    void pose_array_callback(
        const geometry_msgs::msg::PoseArray::ConstSharedPtr msg);

    message_filters::Subscriber<geometry_msgs::msg::PoseStamped> pose_sub_;
    message_filters::Subscriber<geometry_msgs::msg::PoseArray> pose_array_sub_;
    std::shared_ptr<tf2_ros::MessageFilter<geometry_msgs::msg::PoseStamped>>
        tf2_filter_pose_;
    std::shared_ptr<tf2_ros::MessageFilter<geometry_msgs::msg::PoseArray>>
        tf2_filter_pose_array_;

    std::string target_frame_;
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;
};

}  // namespace vortex::filtering

#endif  // IPDA_POSE_FILTERING_ROS_HPP
