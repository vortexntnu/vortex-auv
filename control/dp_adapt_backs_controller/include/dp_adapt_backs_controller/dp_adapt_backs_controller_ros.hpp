#ifndef DP_ADAPT_BACKS_CONTROLLER__DP_ADAPT_BACKS_CONTROLLER_ROS_HPP_
#define DP_ADAPT_BACKS_CONTROLLER__DP_ADAPT_BACKS_CONTROLLER_ROS_HPP_

#include <chrono>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <vortex/utils/types.hpp>
#include <vortex_msgs/msg/operation_mode.hpp>
#include <vortex_msgs/msg/reference_filter.hpp>
#include <vortex_msgs/srv/get_operation_mode.hpp>
#include "dp_adapt_backs_controller/dp_adapt_backs_controller.hpp"
#include "dp_adapt_backs_controller/typedefs.hpp"
#include "typedefs.hpp"

namespace vortex::control {

// @brief Class for the DP Adaptive Backstepping controller node
class DPAdaptBacksControllerNode : public rclcpp::Node {
   public:
    explicit DPAdaptBacksControllerNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

   private:
    // @brief Client for the GetOperationMode service
    rclcpp::Client<vortex_msgs::srv::GetOperationMode>::SharedPtr
        get_operation_mode_client_;

    // @brief Callback function for the killswitch topic
    // @param msg: Bool message containing the killswitch status
    void killswitch_callback(const std_msgs::msg::Bool::SharedPtr msg);

    // @brief Callback function for the software mode topic
    // @param msg: String message containing the software mode
    void operation_mode_callback(
        const vortex_msgs::msg::OperationMode::SharedPtr msg);

    // @brief Callback function for the pose topic
    // @param msg: PoseWithCovarianceStamped message containing the AUV pose
    void pose_callback(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    // @brief Callback function for the twist topic
    // @param msg: TwistWithCovarianceStamped message containing the AUV speed
    void twist_callback(
        const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);

    // @brief Callback function for the control input tau publish
    void publish_tau();

    // @brief set the DP Adaptive Backstepping controller parameters
    void set_adap_params();

    // @brief Set the subscriber and publisher for the node
    void set_subscribers_and_publisher();

    // @brief Initialize the operation mode by calling the GetOperationMode
    // service
    void initialize_operation_mode();

    // @brief Callback function for the guidance topic
    // @param msg: ReferenceFilter message containing the desired vehicle pose
    // and velocity
    void guidance_callback(
        const vortex_msgs::msg::ReferenceFilter::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr killswitch_sub_{};

    rclcpp::Subscription<vortex_msgs::msg::OperationMode>::SharedPtr
        operation_mode_sub_{};

    rclcpp::Subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_{};

    rclcpp::Subscription<
        geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_sub_{};

    rclcpp::Subscription<vortex_msgs::msg::ReferenceFilter>::SharedPtr
        guidance_sub_{};

    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr tau_pub_{};

    rclcpp::TimerBase::SharedPtr tau_pub_timer_{};

    std::chrono::milliseconds time_step_{};

    vortex::utils::types::PoseEuler pose_;

    vortex::utils::types::PoseEuler pose_d_;

    vortex::utils::types::Twist twist_;

    std::unique_ptr<DPAdaptBacksController> dp_adapt_backs_controller_{};

    bool killswitch_on_{true};

    vortex::utils::types::Mode operation_mode_{
        vortex::utils::types::Mode::manual};
};

}  // namespace vortex::control

#endif  // DP_ADAPT_BACKS_CONTROLLER__DP_ADAPT_BACKS_CONTROLLER_ROS_HPP_
