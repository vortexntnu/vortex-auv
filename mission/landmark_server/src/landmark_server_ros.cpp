#include "landmark_server/landmark_server_ros.hpp"
#include <spdlog/spdlog.h>
#include <rclcpp_action/create_client.hpp>
#include <rclcpp_action/create_server.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <vortex/utils/ros/qos_profiles.hpp>
#include <vortex_msgs/action/detail/landmark_convergence__struct.hpp>

namespace vortex::mission {

LandmarkServerNode::LandmarkServerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("landmark_server_node", options) {
    setup_ros_communicators();
}

void LandmarkServerNode::setup_ros_communicators() {
    std::string landmark_topic =
        this->declare_parameter<std::string>("topics.landmarks");
    std::string reference_pose_topic =
        this->declare_parameter<std::string>("topics.reference_pose");
    auto qos_sensor_profile =
        vortex::utils::qos_profiles::sensor_data_profile(5);
    landmark_array_sub_ =
        this->create_subscription<vortex_msgs::msg::LandmarkArray>(
            landmark_topic, qos_sensor_profile,
            std::bind(&LandmarkServerNode::landmark_callback, this,
                      std::placeholders::_1));
    reference_pose_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>(
            reference_pose_topic, qos_sensor_profile);
    create_polling_action_server();
    create_convergence_action_server();
    create_reference_action_client();
}

void LandmarkServerNode::create_polling_action_server() {
    std::string landmark_polling_action_name =
        this->declare_parameter<std::string>(
            "actions_servers.landmark_polling");
    landmark_polling_server_ =
        rclcpp_action::create_server<vortex_msgs::action::LandmarkPolling>(
            this, landmark_polling_action_name,

            [this](auto goal_id, auto goal) {
                return handle_landmark_polling_goal(goal_id, goal);
            },

            [this](auto goal_id) {
                return handle_landmark_polling_cancel(goal_id);
            },

            [this](auto goal_handle) {
                return handle_landmark_polling_accepted(goal_handle);
            });
}

void LandmarkServerNode::create_convergence_action_server() {
    std::string landmark_convergence_action_name =
        this->declare_parameter<std::string>(
            "actions_servers.landmark_convergence");
    landmark_convergence_server_ =
        rclcpp_action::create_server<vortex_msgs::action::LandmarkConvergence>(
            this, landmark_convergence_action_name,

            [this](auto goal_id, auto goal) {
                return handle_landmark_convergence_goal(goal_id, goal);
            },

            [this](auto goal_id) {
                return handle_landmark_convergence_cancel(goal_id);
            },

            [this](auto goal_handle) {
                return handle_landmark_convergence_accepted(goal_handle);
            });
}

void LandmarkServerNode::create_reference_action_client() {
    std::string reference_action_name =
        this->declare_parameter<std::string>("actions.reference_filter");
    reference_filter_client_ = rclcpp_action::create_client<
        vortex_msgs::action::ReferenceFilterWaypoint>(this,
                                                      reference_action_name);
    if (!reference_filter_client_->wait_for_action_server(
            std::chrono::seconds(3))) {
        spdlog::warn("ReferenceFilter server not ready");
    }
}

}  // namespace vortex::mission

RCLCPP_COMPONENTS_REGISTER_NODE(vortex::mission::LandmarkServerNode)
