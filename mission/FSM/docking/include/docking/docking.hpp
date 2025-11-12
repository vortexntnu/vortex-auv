#ifndef DOCKING__DOCKING_HPP_
#define DOCKING__DOCKING_HPP_

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

#include <spdlog/spdlog.h>

namespace docking_fsm {
using PoseStamped = geometry_msgs::msg::PoseStamped;
using Pose = geometry_msgs::msg::Pose;
using PointStamped = geometry_msgs::msg::PointStamped;

using FindDockingAction = vortex_msgs::action::FilteredPose;
using ApproachDockingAction = vortex_msgs::action::ReferenceFilterWaypoint;
using GoAboveDockingAction = vortex_msgs::action::ReferenceFilterWaypoint;
using ConvergeDockingAction = vortex_msgs::action::ReferenceFilterWaypoint;
using ReturnHomeAction = vortex_msgs::action::ReferenceFilterWaypoint;

}  // namespace docking_fsm

class FindDockingStationState
    : public yasmin_ros::ActionState<docking_fsm::FindDockingAction> {
   public:
    FindDockingStationState(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

    docking_fsm::FindDockingAction::Goal create_goal_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

    std::string response_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        docking_fsm::FindDockingAction::Result::SharedPtr response);

    void print_feedback(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        std::shared_ptr<const docking_fsm::FindDockingAction::Feedback>
            feedback);
};

class ApproachDockingStationState
    : public yasmin_ros::ActionState<docking_fsm::ApproachDockingAction> {
   public:
    ApproachDockingStationState(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

    docking_fsm::ApproachDockingAction::Goal create_goal_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

    std::string response_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        docking_fsm::ApproachDockingAction::Result::SharedPtr response);

    void print_feedback(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        std::shared_ptr<const docking_fsm::ApproachDockingAction::Feedback>
            feedback);
};

class GoAboveDockingStationState
    : public yasmin_ros::ActionState<docking_fsm::GoAboveDockingAction> {
   public:
    GoAboveDockingStationState(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

    docking_fsm::GoAboveDockingAction::Goal create_goal_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

    std::string response_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        docking_fsm::GoAboveDockingAction::Result::SharedPtr response);

    void print_feedback(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        std::shared_ptr<const docking_fsm::GoAboveDockingAction::Feedback>
            feedback);
};

std::string DockedState(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

class ReturnHomeState
    : public yasmin_ros::ActionState<docking_fsm::ReturnHomeAction> {
   public:
    explicit ReturnHomeState(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

    docking_fsm::ReturnHomeAction::Goal create_goal_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

    std::string response_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        docking_fsm::ReturnHomeAction::Result::SharedPtr response);

    void print_feedback(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        std::shared_ptr<const docking_fsm::ReturnHomeAction::Feedback>
            feedback);
};

std::string AbortState(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

std::string ErrorState(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

class ConvergeDockingStationState
    : public yasmin_ros::ActionState<docking_fsm::ConvergeDockingAction> {
   public:
    ConvergeDockingStationState(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

    docking_fsm::ConvergeDockingAction::Goal create_goal_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

    std::string response_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        docking_fsm::ConvergeDockingAction::Result::SharedPtr response);

    void print_feedback(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        std::shared_ptr<const docking_fsm::ConvergeDockingAction::Feedback>
            feedback);
};

std::shared_ptr<yasmin::StateMachine> create_state_machines();

void add_states(std::shared_ptr<yasmin::StateMachine> sm,
                std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

auto initialize_blackboard();

#endif  // DOCKING__DOCKING_HPP_
