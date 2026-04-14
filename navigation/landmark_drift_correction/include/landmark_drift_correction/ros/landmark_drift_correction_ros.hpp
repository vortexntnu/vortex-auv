#ifndef LANDMARK_DRIFT_CORRECTION__ROS__LANDMARK_DRIFT_CORRECTION_ROS_HPP_
#define LANDMARK_DRIFT_CORRECTION__ROS__LANDMARK_DRIFT_CORRECTION_ROS_HPP_

#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <landmark_drift_correction/lib/drift_corrector.hpp>

namespace vortex::navigation::drift_correction {

class LandmarkDriftCorrectionNode : public rclcpp::Node {
   public:
    explicit LandmarkDriftCorrectionNode(const rclcpp::NodeOptions& options);
    ~LandmarkDriftCorrectionNode() = default;

   private:
    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
    void keyframe_timer_callback();

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        graph_pub_;
    rclcpp::TimerBase::SharedPtr keyframe_timer_;

    DriftCorrector drift_corrector_;

    Eigen::Vector3d latest_pos_{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond latest_rot_{Eigen::Quaterniond::Identity()};
    double latest_stamp_{0.0};
    bool has_odom_{false};

    std::string odom_frame_;
};

}  // namespace vortex::navigation::drift_correction

#endif  // LANDMARK_DRIFT_CORRECTION__ROS__LANDMARK_DRIFT_CORRECTION_ROS_HPP_
