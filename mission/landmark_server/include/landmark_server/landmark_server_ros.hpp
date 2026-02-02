#ifndef LANDMARK_SERVER__LANDMARK_SERVER_ROS_HPP_
#define LANDMARK_SERVER__LANDMARK_SERVER_ROS_HPP_

#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vortex_msgs/msg/landmark_array.hpp>

namespace vortex::mission {

class LandmarkServerNode : public rclcpp::Node {
   public:
    explicit LandmarkServerNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~LandmarkServerNode() = default;

   private:
    void create_polling_action();

    void create_convergance_action();

    void create_reference_action();

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
        reference_pose_pub_;

    rclcpp::Subscription<vortex_msgs::msg::LandmarkArray>::SharedPtr
        landmark_array_sub_;
};

}  // namespace vortex::mission

#endif  // LANDMARK_SERVER__LANDMARK_SERVER_ROS_HPP_
