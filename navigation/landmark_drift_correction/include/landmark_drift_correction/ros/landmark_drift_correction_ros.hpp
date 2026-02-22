#ifndef LANDMARK_DRIFT_CORRECTION__ROS__LANDMARK_DRIFT_CORRECTION_ROS_HPP_
#define LANDMARK_DRIFT_CORRECTION__ROS__LANDMARK_DRIFT_CORRECTION_ROS_HPP_

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_ros/transform_broadcaster.h>
#include <vortex_msgs/msg/landmark_array.hpp>
#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <pose_filtering/lib/pose_track_manager.hpp>

namespace vortex::navigation::drift_correction {

class LandmarkDriftCorrectionNode : public rclcpp::Node {
   public:
    explicit LandmarkDriftCorrectionNode(const rclcpp::NodeOptions& options);

    ~LandmarkDriftCorrectionNode() = default;

   private:
    void setup_publishers_and_subscribers();

    void setup_drift_corrector();

    void landmark_callback(
        const vortex_msgs::msg::LandmarkArray::ConstSharedPtr& msg);

    void timer_callback();

    std::shared_ptr<
        message_filters::Subscriber<vortex_msgs::msg::LandmarkArray>>
        landmark_sub_;

    std::shared_ptr<tf2_ros::MessageFilter<vortex_msgs::msg::LandmarkArray>>
        tf_filter_;

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
        pose_sub_;

    std::unique_ptr<vortex::filtering::PoseTrackManager> track_manager_;

    std::vector<vortex::filtering::Landmark> measurements_;

    rclcpp::TimerBase::SharedPtr filter_timer_;    
    rclcpp::TimerBase::SharedPtr pose_update_timer_;    

    double filter_dt_seconds_{0.0};
    std::string body_frame_;
    std::string filter_frame_;
    std::string corrected_frame_;
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr drift_pose_pub_;

};

}  // namespace vortex::navigation::drift_correction

#endif  // LANDMARK_DRIFT_CORRECTION__ROS__LANDMARK_DRIFT_CORRECTION_ROS_HPP_
