#ifndef VORTEX_PIPELINE_HPP
#define VORTEX_PIPELINE_HPP

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

namespace pipeline_fsm {
using PoseStamped = geometry_msgs::msg::PoseStamped;
using Pose = geometry_msgs::msg::Pose;
using PointStamped = geometry_msgs::msg::PointStamped;

using FindPipelineAction = vortex_msgs::action::FilteredPose;
using ApproachPipelineAction = vortex_msgs::action::ReferenceFilterWaypoint;
using FollowPipelineAction = vortex_msgs::action::ReferenceFilterWaypoint;
using ConvergeDockingAction = vortex_msgs::action::ReferenceFilterWaypoint;
using ReturnHomeAction = vortex_msgs::action::ReferenceFilterWaypoint;

}  // namespace pipeline_fsm

class FindPipelineState
    : public yasmin_ros::ActionState<pipeline_fsm::FindPipelineAction> {
   public:
    FindPipelineState(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

    pipeline_fsm::FindPipelineAction::Goal create_goal_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

    std::string response_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        pipeline_fsm::FindPipelineAction::Result::SharedPtr response);

    void print_feedback(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        std::shared_ptr<const pipeline_fsm::FindPipelineAction::Feedback>
            feedback);
};

class ApproachPipelineState
    : public yasmin_ros::ActionState<pipeline_fsm::ApproachPipelineAction> {
   public:
    ApproachPipelineState(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

    pipeline_fsm::ApproachPipelineAction::Goal create_goal_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

    std::string response_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        pipeline_fsm::ApproachPipelineAction::Result::SharedPtr response);

    void print_feedback(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        std::shared_ptr<const pipeline_fsm::ApproachPipelineAction::Feedback>
            feedback);
};

class FollowPipelineState
    : public yasmin_ros::ActionState<pipeline_fsm::FollowPipelineAction> {
   public:
    FollowPipelineState(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

    pipeline_fsm::FollowPipelineAction::Goal create_goal_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

    std::string response_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        pipeline_fsm::FollowPipelineAction::Result::SharedPtr response);

    void print_feedback(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        std::shared_ptr<const pipeline_fsm::FollowPipelineAction::Feedback>
            feedback);
};

class ReturnHomeState
    : public yasmin_ros::ActionState<pipeline_fsm::ReturnHomeAction> {
   public:
    ReturnHomeState(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

    pipeline_fsm::ReturnHomeAction::Goal create_goal_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

    std::string response_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        pipeline_fsm::ReturnHomeAction::Result::SharedPtr response);

    void print_feedback(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        std::shared_ptr<const pipeline_fsm::ReturnHomeAction::Feedback>
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
