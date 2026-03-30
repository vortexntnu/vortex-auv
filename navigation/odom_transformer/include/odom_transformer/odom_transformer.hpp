#ifndef ODOM_TRANSFORMER__ODOM_TRANSFORMER_HPP_
#define ODOM_TRANSFORMER__ODOM_TRANSFORMER_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_eigen/tf2_eigen.hpp>

class OdomTransformer : public rclcpp::Node {
   public:
    explicit OdomTransformer(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

   private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void lookup_static_transforms();
    void complete_initialization();

    std::string frame(const std::string& name) const {
        return frame_prefix_.empty() ? name : frame_prefix_ + "/" + name;
    }

    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr tf_timer_;

    // Pub / Sub
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
        pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
        twist_pub_;

    // Transform from base_link to sensor_link
    Eigen::Matrix3d R_base_sensor_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_base_sensor_ = Eigen::Vector3d::Zero();

    std::string frame_prefix_;
    std::string sensor_frame_;
    bool tf_loaded_{false};
    bool publish_tf_{false};
    bool publish_pose_{false};
    bool publish_twist_{false};
};

#endif  // ODOM_TRANSFORMER__ODOM_TRANSFORMER_HPP_
