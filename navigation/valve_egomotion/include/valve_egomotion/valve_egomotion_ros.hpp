#pragma once

#include <array>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "valve_egomotion/valve_egomotion.hpp"

namespace valve_egomotion {

/**
 * ROS2 node:
 * - subscribes to PoseArray of marker poses in camera frame
 * - looks up T_base<-cam
 * - defines an initial anchor marker0 in base frame
 * - computes T_ref<-base = T_base<-marker0 * (T_base<-marker_now)^-1
 * - smoothes in a window and publishes PoseWithCovarianceStamped in ref_frame
 */
class ValveEgomotionNode final : public rclcpp::Node {
   public:
    explicit ValveEgomotionNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

   private:
    void onMarkers(const geometry_msgs::msg::PoseArray::SharedPtr msg);
    void onPublishTimer();

    // Params
    std::string valve_topic_;
    std::string ref_frame_;
    std::string base_frame_;
    std::string cam_frame_;
    bool publish_tf_ = false;

    double pub_rate_hz_ = 30.0;
    double lost_timeout_sec_ = 0.5;

    // TF
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // ROS I/O
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_marker_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
        pub_pose_cov_;
    rclcpp::TimerBase::SharedPtr pub_timer_;

    // Estimator
    SlidingWindowSO3Mean smoother_;

    // Anchor and timing
    bool have_ref_ = false;
    tf2::Transform T_base_marker0_;
    rclcpp::Time last_det_stamp_;
    rclcpp::Time last_pub_stamp_;
};

}  // namespace valve_egomotion