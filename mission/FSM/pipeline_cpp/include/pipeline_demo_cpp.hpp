#ifndef VORTEX_PIPELINE_HPP
#define VORTEX_PIPELINE_HPP

#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include <vortex_msgs/action/find_dock.hpp>
#include <vortex_msgs/action/go_to_waypoint.hpp>
#include <vortex_msgs/action/navigate_waypoints.hpp>

#include <yasmin/cb_state.hpp>
#include <yasmin/logs.hpp>
#include <yasmin/state_machine.hpp>
#include <yasmin_ros/action_state.hpp>
#include <yasmin_ros/basic_outcomes.hpp>
#include <yasmin_ros/ros_logs.hpp>
#include <yasmin_ros/yasmin_node.hpp>
#include <yasmin_viewer/yasmin_viewer_pub.hpp>

using std::placeholders::_1;
using std::placeholders::_2;
using FindDock = vortex_msgs::action::FindDock;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using Pose = geometry_msgs::msg::Pose;
using NavigateWaypoints = vortex_msgs::action::NavigateWaypoints;
using namespace yasmin;

class FindPipelineState : public yasmin_ros::ActionState<FindDock> {
   public:
    FindPipelineState();

    FindDock::Goal create_goal_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

    std::string response_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        FindDock::Result::SharedPtr response);

    void print_feedback(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        std::shared_ptr<const FindDock::Feedback> feedback);
};

class GoToStartPipelineState : public yasmin_ros::ActionState<NavigateWaypoints> {
   public:
    GoToStartPipelineState();

    NavigateWaypoints::Goal create_goal_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

    std::string response_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        NavigateWaypoints::Result::SharedPtr response);

    void print_feedback(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        std::shared_ptr<const NavigateWaypoints::Feedback> feedback);
};

class FollowPipelineState : public yasmin_ros::ActionState<NavigateWaypoints> {
   public:
    FollowPipelineState();

    NavigateWaypoints::Goal create_goal_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

    std::string response_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        NavigateWaypoints::Result::SharedPtr response);

    void print_feedback(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        std::shared_ptr<const NavigateWaypoints::Feedback> feedback);
};

class ReturnHomeState : public yasmin_ros::ActionState<NavigateWaypoints> {
   public:
    ReturnHomeState();

    NavigateWaypoints::Goal create_goal_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

    std::string response_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        NavigateWaypoints::Result::SharedPtr response);

    void print_feedback(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        std::shared_ptr<const NavigateWaypoints::Feedback> feedback);
};

std::string AbortState(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);


std::string ErrorState(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

std::shared_ptr<yasmin::StateMachine> create_state_machine();

void add_states(std::shared_ptr<yasmin::StateMachine> sm);

auto initialize_blackboard();

#endif
