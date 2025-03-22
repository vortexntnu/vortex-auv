#ifndef VORTEX_DOCKING_HPP
#define VORTEX_DOCKING_HPP

#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include <vortex_msgs/action/filtered_pose.hpp>
#include <vortex_msgs/action/los_guidance.hpp>
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

namespace docking_fsm {
using FilteredPose = vortex_msgs::action::FilteredPose;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using Pose = geometry_msgs::msg::Pose;
using PointStamped = geometry_msgs::msg::PointStamped;
using ReferenceFilterWaypoint = vortex_msgs::action::ReferenceFilterWaypoint;
using ReferenceFilter = vortex_msgs::msg::ReferenceFilter;
using LOSGuidance = vortex_msgs::action::LOSGuidance;
}  // namespace docking_fsm

class FindDockingStationState
    : public yasmin_ros::ActionState<docking_fsm::FilteredPose> {
   public:
    FindDockingStationState();

    docking_fsm::FilteredPose::Goal create_goal_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

    std::string response_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        docking_fsm::FilteredPose::Result::SharedPtr response);

    void print_feedback(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        std::shared_ptr<const docking_fsm::FilteredPose::Feedback> feedback);
};

class ApproachDockingStationState
    : public yasmin_ros::ActionState<docking_fsm::LOSGuidance> {
   public:
    ApproachDockingStationState();

    docking_fsm::LOSGuidance::Goal create_goal_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

    std::string response_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        docking_fsm::LOSGuidance::Result::SharedPtr response);

    void print_feedback(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        std::shared_ptr<const docking_fsm::LOSGuidance::Feedback> feedback);
};

class GoAboveDockingStationState
    : public yasmin_ros::ActionState<docking_fsm::ReferenceFilterWaypoint> {
   public:
    GoAboveDockingStationState(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

    docking_fsm::ReferenceFilterWaypoint::Goal create_goal_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

    std::string response_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        docking_fsm::ReferenceFilterWaypoint::Result::SharedPtr response);

    void print_feedback(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        std::shared_ptr<const docking_fsm::ReferenceFilterWaypoint::Feedback>
            feedback);
};

std::string DockedState(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

class ReturnHomeState
    : public yasmin_ros::ActionState<docking_fsm::ReferenceFilterWaypoint> {
   public:
    ReturnHomeState();

    docking_fsm::ReferenceFilterWaypoint::Goal create_goal_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

    std::string response_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        docking_fsm::ReferenceFilterWaypoint::Result::SharedPtr response);

    void print_feedback(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        std::shared_ptr<const docking_fsm::ReferenceFilterWaypoint::Feedback>
            feedback);
};

std::string AbortState(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

std::string ErrorState(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

class ConvergeDockingStationState
    : public yasmin_ros::ActionState<docking_fsm::ReferenceFilterWaypoint> {
   public:
    ConvergeDockingStationState();

    docking_fsm::ReferenceFilterWaypoint::Goal create_goal_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

    std::string response_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        docking_fsm::ReferenceFilterWaypoint::Result::SharedPtr response);

    void print_feedback(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        std::shared_ptr<const docking_fsm::ReferenceFilterWaypoint::Feedback>
            feedback);
};

std::shared_ptr<yasmin::StateMachine> create_state_machines();

void add_states(std::shared_ptr<yasmin::StateMachine> sm,
                std::shared_ptr<yasmin::StateMachine> nested_sm);

void add_states_nested(
    std::shared_ptr<yasmin::StateMachine> sm,
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

auto initialize_blackboard();

#endif
