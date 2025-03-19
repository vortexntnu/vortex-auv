#ifndef EKF_POSE_FILTERING_ROS_HPP
#define EKF_POSE_FILTERING_ROS_HPP

#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tuple>
#include <vortex_filtering/vortex_filtering.hpp>

class EKFPoseFilteringNode : public rclcpp::Node {
   public:
    EKFPoseFilteringNode();

    ~EKFPoseFilteringNode() {};

   private:
    void reset_EFK_state(std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    void pose_callback(
        const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_msg);

    void filter_pose(geometry_msgs::msg::PoseStamped& transformed_pose);

    geometry_msgs::msg::Quaternion enu_to_ned_quaternion(
        const geometry_msgs::msg::Quaternion& enu_quat);

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;

    std::string target_frame_;
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

    message_filters::Subscriber<geometry_msgs::msg::PoseStamped> pose_sub_;
    std::shared_ptr<tf2_ros::MessageFilter<geometry_msgs::msg::PoseStamped>>
        tf2_filter_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
        transformed_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
        filtered_pose_pub_;

    using DynMod = vortex::models::ConstantDynamicModel<3>;
    using SensMod = vortex::models::IdentitySensorModel<3, 3>;
    using EKF = vortex::filter::EKF<DynMod, SensMod>;
    using Gauss3d = vortex::prob::Gauss<3>;

    bool first_run_ = true;
    rclcpp::Time previous_time_;

    std::shared_ptr<DynMod> dynamic_model_;
    std::shared_ptr<SensMod> sensor_model_;

    Gauss3d previous_pose_est_;
    Gauss3d object_pose_est_;

    std::string frame_;
    bool enu_orientation_;
};

#endif  // EKF_POSE_FILTERING_ROS_HPP
