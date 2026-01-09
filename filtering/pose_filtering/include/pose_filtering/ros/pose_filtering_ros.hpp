#ifndef POSE_FILTERING__ROS__POSE_FILTERING_ROS_HPP_
#define POSE_FILTERING__ROS__POSE_FILTERING_ROS_HPP_

#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <vortex_msgs/msg/landmark_array.hpp>
#include <vortex_msgs/msg/pose_euler_stamped.hpp>
#include "pose_filtering/lib/pose_track_manager.hpp"

#include <concepts>

namespace vortex::filtering {

using PoseMsgT = geometry_msgs::msg::PoseStamped;

template <typename T>
concept ValidPoseMsg =
    std::same_as<T, geometry_msgs::msg::PoseStamped> ||
    std::same_as<T, geometry_msgs::msg::PoseArray> ||
    std::same_as<T, geometry_msgs::msg::PoseWithCovarianceStamped> ||
    std::same_as<T, vortex_msgs::msg::LandmarkArray>;

static_assert(ValidPoseMsg<PoseMsgT>,
              "PoseMsgT must be a supported pose message type");

class PoseFilteringNode : public rclcpp::Node {
   public:
    explicit PoseFilteringNode(const rclcpp::NodeOptions& options);

    ~PoseFilteringNode() {}

   private:
    void setup_publishers_and_subscribers();

    void setup_track_manager();

    void create_pose_subscription(const std::string& topic_name,
                                  const rmw_qos_profile_t& qos_profile);

    void timer_callback();

    void setup_debug_publishers();

    void publish_meas_debug();

    void publish_state_debug();

    std::shared_ptr<message_filters::Subscriber<PoseMsgT>> subscriber_;

    std::shared_ptr<tf2_ros::MessageFilter<PoseMsgT>> tf_filter_;

    std::string target_frame_;
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;

    rclcpp::Publisher<vortex_msgs::msg::LandmarkArray>::SharedPtr
        landmark_array_pub_;

    rclcpp::TimerBase::SharedPtr pub_timer_;
    double filter_dt_seconds_{0.0};

    std::unique_ptr<PoseTrackManager> track_manager_;

    std::vector<Pose> measurements_;
    bool enu_ned_rotation_{false};

    bool debug_{false};
    rclcpp::Publisher<vortex_msgs::msg::PoseEulerStamped>::SharedPtr
        pose_meas_debug_pub_;
    rclcpp::Publisher<vortex_msgs::msg::PoseEulerStamped>::SharedPtr
        pose_state_debug_pub_;
};

}  // namespace vortex::filtering

#endif  // POSE_FILTERING__ROS__POSE_FILTERING_ROS_HPP_
