#include "landmark_drift_correction/ros/landmark_drift_correction_ros.hpp"
#include "landmark_drift_correction/ros/graph_visualizer.hpp"

#include <spdlog/spdlog.h>
#include <rclcpp_components/register_node_macro.hpp>

namespace vortex::navigation::drift_correction {

LandmarkDriftCorrectionNode::LandmarkDriftCorrectionNode(
    const rclcpp::NodeOptions& options)
    : rclcpp::Node("landmark_drift_correction_node", options) {
    const std::string odom_topic =
        this->declare_parameter<std::string>("topics.odom");
    odom_frame_ = this->declare_parameter<std::string>("odom_frame");

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, rclcpp::QoS(10).best_effort(),
        [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
            this->odom_callback(msg);
        });

    graph_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "graph/visualization", rclcpp::QoS(10));

    keyframe_timer_ = this->create_wall_timer(
        std::chrono::seconds(5),
        std::bind(&LandmarkDriftCorrectionNode::keyframe_timer_callback, this));

    spdlog::info(
        "Landmark Drift Correction Node initialized with odometry topic '{}' "
        "and odometry frame '{}'",
        odom_topic, odom_frame_);
}

void LandmarkDriftCorrectionNode::odom_callback(
    const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    const auto& pos = msg->pose.pose.position;
    const auto& ori = msg->pose.pose.orientation;
    latest_pos_ = {pos.x, pos.y, pos.z};
    latest_rot_ = Eigen::Quaterniond{ori.w, ori.x, ori.y, ori.z}.normalized();
    has_odom_ = true;
}

void LandmarkDriftCorrectionNode::keyframe_timer_callback() {
    if (!has_odom_) {
        spdlog::warn("Waiting for first odometry message...");
        return;
    }

    const double t = this->now().seconds();
    drift_corrector_.addKeyframe(t, latest_pos_, latest_rot_);

    spdlog::info("Keyframe k={} added at t={:.3f}",
                 drift_corrector_.keyframe_count() - 1, t);

    graph_pub_->publish(build_graph_markers(drift_corrector_.keyframes(),
                                            odom_frame_, this->now()));
}

RCLCPP_COMPONENTS_REGISTER_NODE(
    vortex::navigation::drift_correction::LandmarkDriftCorrectionNode)

}  // namespace vortex::navigation::drift_correction
