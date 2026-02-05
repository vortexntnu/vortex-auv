#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vortex_msgs/msg/landmark_array.hpp>
#include <vortex_msgs/msg/landmark.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "visual_egomotion/visual_egomotion.hpp"

namespace visual_egomotion {

/**
 * ROS2 node:
 * - subscribes to PoseArray of marker poses in camera frame
 * - looks up T_base<-cam
 * - defines an initial anchor marker0 in base frame
 * - computes T_ref<-base = T_base<-marker0 * (T_base<-marker_now)^-1
 * - smoothes in a window and publishes PoseWithCovarianceStamped in ref_frame
 */
class VisualEgomotionNode final : public rclcpp::Node {
   public:
    explicit VisualEgomotionNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

   private:
    void onLandmarks(const vortex_msgs::msg::LandmarkArray::SharedPtr msg);
    void onPublishTimer();

    // Params
    std::string visual_topic_;
    std::string ref_frame_;
    std::string base_frame_;
    std::string cam_frame_;
    bool publish_tf_; 
    double pub_rate_hz_; 
    double lost_timeout_sec_;
    rclcpp::Time last_pub_det_stamp;
    
    // Anchor selection
    uint16_t current_marker_id_{0xFFFF};
    size_t multi_skip_count_{0};

    // Anchor and timing
    bool have_ref = false;
    tf2::Transform T_anchor_; // Transform landmark anchor to base
    rclcpp::Time last_det_stamp;
    rclcpp::Time last_pub_stamp;
    
    // TF
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // ROS I/O
    rclcpp::Subscription<vortex_msgs::msg::LandmarkArray>::SharedPtr
        sub_landmarks_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
        pub_pose_cov_;
    rclcpp::TimerBase::SharedPtr pub_timer_;

    // Estimator
    std::unique_ptr<SlidingWindowSO3Mean> smoother_;

    bool have_last_ref_base_ = false;
    tf2::Transform last_T_ref_base_;

    void set_subscribers_and_publisher();
    void set_parameters();

};

}  // namespace visual_egomotion