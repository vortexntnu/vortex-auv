#ifndef PIPELINE__PIPELINE_HPP_
#define PIPELINE__PIPELINE_HPP_

#include <memory>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <vortex_msgs/action/landmark_convergence.hpp>
#include <vortex_msgs/action/landmark_polling.hpp>
#include <vortex_msgs/action/waypoint_manager.hpp>
#include <vortex_msgs/msg/landmark.hpp>
#include <vortex_msgs/msg/waypoint.hpp>
#include <vortex_msgs/srv/send_waypoints.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <yasmin/blackboard.hpp>
#include <yasmin/logs.hpp>
#include <yasmin/state_machine.hpp>
#include <yasmin_ros/action_state.hpp>
#include <yasmin_ros/basic_outcomes.hpp>
#include <yasmin_ros/ros_logs.hpp>
#include <yasmin_ros/yasmin_node.hpp>
#include <yasmin_viewer/yasmin_viewer_pub.hpp>

namespace pipeline_fsm {
using WaypointManagerAction = vortex_msgs::action::WaypointManager;
using WaypointManagerGoalHandle =
    rclcpp_action::ClientGoalHandle<WaypointManagerAction>;
using LandmarkPollingAction = vortex_msgs::action::LandmarkPolling;
using LandmarkConvergenceAction = vortex_msgs::action::LandmarkConvergence;
using SendWaypointsSrv = vortex_msgs::srv::SendWaypoints;
}  // namespace pipeline_fsm

class SearchState
    : public yasmin_ros::ActionState<pipeline_fsm::LandmarkPollingAction> {
   public:
    explicit SearchState(yasmin::Blackboard::SharedPtr blackboard);

    std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;

    pipeline_fsm::LandmarkPollingAction::Goal create_goal_handler(
        yasmin::Blackboard::SharedPtr blackboard);

    std::string response_handler(
        yasmin::Blackboard::SharedPtr blackboard,
        pipeline_fsm::LandmarkPollingAction::Result::SharedPtr response);

   private:
    pipeline_fsm::WaypointManagerGoalHandle::SharedPtr start_waypoint_manager(
        yasmin::Blackboard::SharedPtr blackboard);

    void stop_waypoint_manager(
        pipeline_fsm::WaypointManagerGoalHandle::SharedPtr handle);

    void push_initial_waypoint(yasmin::Blackboard::SharedPtr blackboard);

    rclcpp_action::Client<pipeline_fsm::WaypointManagerAction>::SharedPtr
        wm_client_;
    rclcpp::Client<pipeline_fsm::SendWaypointsSrv>::SharedPtr
        wp_service_client_;
};

class LandmarkConvergeState
    : public yasmin_ros::ActionState<pipeline_fsm::LandmarkConvergenceAction> {
   public:
    explicit LandmarkConvergeState(yasmin::Blackboard::SharedPtr blackboard);

    pipeline_fsm::LandmarkConvergenceAction::Goal create_goal_handler(
        yasmin::Blackboard::SharedPtr blackboard);
};

std::shared_ptr<yasmin::Blackboard> initialize_blackboard();

#endif  // PIPELINE__PIPELINE_HPP_
