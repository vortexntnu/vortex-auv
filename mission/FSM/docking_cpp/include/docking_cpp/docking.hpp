#ifndef VORTEX_DOCKING_HPP
#define VORTEX_DOCKING_HPP

#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include <vortex_msgs/action/filtered_pose.hpp>
#include <vortex_msgs/action/los_guidance.hpp>
#include <vortex_msgs/action/navigate_waypoints.hpp>
#include <vortex_msgs/action/reference_filter_waypoint.hpp>

#include <std_msgs/msg/string.hpp>
#include <yasmin/cb_state.hpp>
#include <yasmin/logs.hpp>
#include <yasmin/state_machine.hpp>
#include <yasmin_ros/action_state.hpp>
#include <yasmin_ros/basic_outcomes.hpp>
#include <yasmin_ros/ros_logs.hpp>
#include <yasmin_ros/yasmin_node.hpp>
#include <yasmin_viewer/yasmin_viewer_pub.hpp>

using FilteredPose = vortex_msgs::action::FilteredPose;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using Pose = geometry_msgs::msg::Pose;
using PointStamped = geometry_msgs::msg::PointStamped;
using ReferenceFilterWaypoint = vortex_msgs::action::ReferenceFilterWaypoint;
using ReferenceFilter = vortex_msgs::msg::ReferenceFilter;
using NavigateWaypoints = vortex_msgs::action::NavigateWaypoints;
using LOSGuidance = vortex_msgs::action::LOSGuidance;

class FindDockingStationState : public yasmin_ros::ActionState<FilteredPose> {
   public:
    FindDockingStationState();

    FilteredPose::Goal create_goal_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

    std::string response_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        FilteredPose::Result::SharedPtr response);

    void print_feedback(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        std::shared_ptr<const FilteredPose::Feedback> feedback);
};

class ApproachDockingStationState
    : public yasmin_ros::ActionState<LOSGuidance> {
   public:
    ApproachDockingStationState();

    LOSGuidance::Goal create_goal_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

    std::string response_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        LOSGuidance::Result::SharedPtr response);

    void print_feedback(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        std::shared_ptr<const LOSGuidance::Feedback> feedback);
};

class GoAboveDockingStationState
    : public yasmin_ros::ActionState<ReferenceFilterWaypoint> {
   public:
    GoAboveDockingStationState(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

    ReferenceFilterWaypoint::Goal create_goal_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

    std::string response_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        ReferenceFilterWaypoint::Result::SharedPtr response);

    void print_feedback(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        std::shared_ptr<const ReferenceFilterWaypoint::Feedback> feedback);
};

std::string DockedState(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

class ReturnHomeState
    : public yasmin_ros::ActionState<ReferenceFilterWaypoint> {
   public:
    ReturnHomeState();

    ReferenceFilterWaypoint::Goal create_goal_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

    std::string response_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        ReferenceFilterWaypoint::Result::SharedPtr response);

    void print_feedback(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        std::shared_ptr<const ReferenceFilterWaypoint::Feedback> feedback);
};

std::string AbortState(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

std::string ErrorState(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

class ConvergeDockingStationState
    : public yasmin_ros::ActionState<ReferenceFilterWaypoint> {
   public:
    ConvergeDockingStationState();

    ReferenceFilterWaypoint::Goal create_goal_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

    std::string response_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        ReferenceFilterWaypoint::Result::SharedPtr response);

    void print_feedback(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        std::shared_ptr<const ReferenceFilterWaypoint::Feedback> feedback);
};

std::shared_ptr<yasmin::StateMachine> create_state_machines();

void add_states(std::shared_ptr<yasmin::StateMachine> sm,
                std::shared_ptr<yasmin::StateMachine> nested_sm);

void add_states_nested(
    std::shared_ptr<yasmin::StateMachine> sm,
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

auto initialize_blackboard();

#endif
