#ifndef VORTEX_STRUCTURE_HPP
#define VORTEX_STRUCTURE_HPP

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

namespace structure_fsm {
using PoseStamped = geometry_msgs::msg::PoseStamped;
using Pose = geometry_msgs::msg::Pose;
using PointStamped = geometry_msgs::msg::PointStamped;

using DetectValveAction = vortex_msgs::action::FilteredPose;
using ApproachValveAction = vortex_msgs::action::ReferenceFilterWaypoint;
using UseGripperAction = vortex_msgs::action::ReferenceFilterWaypoint;
using CheckValveTurnedAction = vortex_msgs::action::ReferenceFilterWaypoint;
using GoBackAction = vortex_msgs::action::ReferenceFilterWaypoint;

}  // namespace structure_fsm

class DetectValveState
    : public yasmin_ros::ActionState<structure_fsm::DetectValveAction> {
   public:
    DetectValveState(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

    structure_fsm::DetectValveAction::Goal create_goal_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

    std::string response_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        structure_fsm::DetectValveAction::Result::SharedPtr response);

    void print_feedback(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        std::shared_ptr<const structure_fsm::DetectValveAction::Feedback>
            feedback);
};

class ApproachValveState
    : public yasmin_ros::ActionState<structure_fsm::ApproachValveAction> {
   public:
    ApproachValveState(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

    structure_fsm::ApproachValveAction::Goal create_goal_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

    std::string response_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        structure_fsm::ApproachValveAction::Result::SharedPtr response);

    void print_feedback(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        std::shared_ptr<const structure_fsm::ApproachValveAction::Feedback>
            feedback);
};

class UseGripperState
    : public yasmin_ros::ActionState<structure_fsm::UseGripperAction> {
   public:
    UseGripperState(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

    structure_fsm::UseGripperAction::Goal create_goal_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

    std::string response_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        structure_fsm::UseGripperAction::Result::SharedPtr response);

    void print_feedback(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        std::shared_ptr<const structure_fsm::UseGripperAction::Feedback>
            feedback);
};

class GoBackState
    : public yasmin_ros::ActionState<structure_fsm::GoBackAction> {
   public:
    GoBackState(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

    structure_fsm::GoBackAction::Goal create_goal_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

    std::string response_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        structure_fsm::GoBackAction::Result::SharedPtr response);

    void print_feedback(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        std::shared_ptr<const structure_fsm::GoBackAction::Feedback> feedback);
};

class CheckValveTurnedState
    : public yasmin_ros::ActionState<structure_fsm::CheckValveTurnedAction> {
   public:
    CheckValveTurnedState(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

    structure_fsm::CheckValveTurnedAction::Goal create_goal_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

    std::string response_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        structure_fsm::CheckValveTurnedAction::Result::SharedPtr response);

    void print_feedback(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        std::shared_ptr<const structure_fsm::CheckValveTurnedAction::Feedback>
            feedback);
};

std::string AbortState(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

std::string ErrorState(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

std::shared_ptr<yasmin::StateMachine> create_state_machines();

void add_states(std::shared_ptr<yasmin::StateMachine> sm,
                std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

auto initialize_blackboard();

#endif
