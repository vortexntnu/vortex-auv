// transform_publisher.hpp
#ifndef TRANSFORM_PUBLISHER_HPP_
#define TRANSFORM_PUBLISHER_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class TransformPublisher : public rclcpp::Node {
   public:
    TransformPublisher();

   private:
    void poseCallback(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    rclcpp::Subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    geometry_msgs::msg::TransformStamped static_transform_;

    bool transform_received_;
};

#endif  // TRANSFORM_PUBLISHER_HPP_
