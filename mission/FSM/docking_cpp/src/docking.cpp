// Copyright (C) 2023  Miguel Ángel González Santamarta

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include <iostream>
#include <memory>
#include <string>

#include <action_tutorials_interfaces/action/fibonacci.hpp>
#include <vortex_msgs/action/find_dock.hpp>
#include <vortex_msgs/action/go_to_waypoint.hpp>

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
using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
using FindDock = vortex_msgs::action::FindDock;
using GoToWaypoint = vortex_msgs::action::GoToWaypoint;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using namespace yasmin;

std::string
print_result(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {

  auto fibo_res = blackboard->get<std::vector<int>>("sum");

  fprintf(stderr, "Result received:");

  for (auto ele : fibo_res) {
    fprintf(stderr, " %d,", ele);
  }

  fprintf(stderr, "\n");

  return yasmin_ros::basic_outcomes::SUCCEED;
}

class FibonacciState : public yasmin_ros::ActionState<Fibonacci> {

public:
  FibonacciState()
      : yasmin_ros::ActionState<Fibonacci>(

            "/fibonacci", // action name

            // # cb to create the goal
            std::bind(&FibonacciState::create_goal_handler, this, _1),
            // # cb to process the response

            std::bind(&FibonacciState::response_handler, this, _1, _2),

            // cb to process the feedback
            std::bind(&FibonacciState::print_feedback, this, _1, _2)) {};

  Fibonacci::Goal create_goal_handler(
      std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {

    auto goal = Fibonacci::Goal();
    goal.order = blackboard->get<int>("n");

    return goal;
  }

  std::string
  response_handler(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
                   Fibonacci::Result::SharedPtr response) {

    blackboard->set<std::vector<int>>("sum", response->sequence);

    return yasmin_ros::basic_outcomes::SUCCEED;
  }

  void
  print_feedback(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
                 std::shared_ptr<const Fibonacci::Feedback> feedback) {
    (void)blackboard;

    std::stringstream ss;
    ss << "Next number in sequence received: ";
    for (auto number : feedback->partial_sequence) {
      ss << number << " ";
    }

    fprintf(stderr, "%s\n", ss.str().c_str());
  }
};

class FindDockState : public yasmin_ros::ActionState<FindDock> {
public:
  FindDockState() : yasmin_ros::ActionState<FindDock>(
                        "/find_dock",
                        std::bind(&FindDockState::create_goal_handler, this, _1),
                        std::bind(&FindDockState::response_handler, this, _1, _2),
                        std::bind(&FindDockState::print_feedback, this, _1, _2)) {};

  FindDock::Goal create_goal_handler(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    auto goal = FindDock::Goal();
    goal.start_search = blackboard->get<bool>("start_search");
    return goal;
  }

  std::string response_handler(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard, FindDock::Result::SharedPtr response) {
    blackboard->set<PoseStamped>("dock_pose", response->dock_pose);
    return yasmin_ros::basic_outcomes::SUCCEED;
  }

  void print_feedback(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard, std::shared_ptr<const FindDock::Feedback> feedback) {
    (void)blackboard;
    fprintf(stderr, "Time elapsed: %f\n",
            feedback->time_elapsed);
  }
};

class GoToDockState : public yasmin_ros::ActionState<GoToWaypoint> {
public:
  GoToDockState() : yasmin_ros::ActionState<GoToWaypoint>(
                        "/go_to_dock",
                        std::bind(&GoToDockState::create_goal_handler, this, _1),
                        std::bind(&GoToDockState::response_handler, this, _1, _2),
                        std::bind(&GoToDockState::print_feedback, this, _1, _2)) {};

  GoToWaypoint::Goal create_goal_handler(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    auto goal = GoToWaypoint::Goal();
    goal.waypoint = blackboard->get<PoseStamped>("dock_pose");
    return goal;
  }

  std::string response_handler(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard, GoToWaypoint::Result::SharedPtr response) {
    (void)blackboard;
    (void)response;
    return yasmin_ros::basic_outcomes::SUCCEED;
  }

  void print_feedback(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard, std::shared_ptr<const GoToWaypoint::Feedback> feedback) {
    (void)blackboard;
    fprintf(stderr, "Current position: x = %f, y = %f, z = %f\n",
            feedback->current_pose.pose.position.x,
            feedback->current_pose.pose.position.y,
            feedback->current_pose.pose.position.z);
  }
};

class DockState : public yasmin_ros::ActionState<GoToWaypoint> {
public:
  DockState() : yasmin_ros::ActionState<GoToWaypoint>(
                    "/dock",
                    std::bind(&DockState::create_goal_handler, this, _1),
                    std::bind(&DockState::response_handler, this, _1, _2),
                    std::bind(&DockState::print_feedback, this, _1, _2)) {};

  GoToWaypoint::Goal create_goal_handler(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    auto goal = GoToWaypoint::Goal();
    goal.waypoint = blackboard->get<PoseStamped>("dock_pose");
    return goal;
  }

  std::string response_handler(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard, GoToWaypoint::Result::SharedPtr response) {
    (void)blackboard;
    (void)response;
    return yasmin_ros::basic_outcomes::SUCCEED;
  }

  void print_feedback(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard, std::shared_ptr<const GoToWaypoint::Feedback> feedback) {
    (void)blackboard;
    fprintf(stderr, "Current position: x = %f, y = %f, z = %f\n",
            feedback->current_pose.pose.position.x,
            feedback->current_pose.pose.position.y,
            feedback->current_pose.pose.position.z);
  }
};

std::string
DockedState(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
  (void)blackboard;
  return yasmin_ros::basic_outcomes::SUCCEED;
};

class ReturnHomeState : public yasmin_ros::ActionState<GoToWaypoint> {
public:
  ReturnHomeState() : yasmin_ros::ActionState<GoToWaypoint>(
                          "/return_home",
                          std::bind(&ReturnHomeState::create_goal_handler, this, _1),
                          std::bind(&ReturnHomeState::response_handler, this, _1, _2),
                          std::bind(&ReturnHomeState::print_feedback, this, _1, _2)) {};

  GoToWaypoint::Goal create_goal_handler(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    auto goal = GoToWaypoint::Goal();
    goal.waypoint = blackboard->get<PoseStamped>("start_pose");
    return goal;
  }

  std::string response_handler(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard, GoToWaypoint::Result::SharedPtr response) {
    (void)blackboard;
    (void)response;
    return yasmin_ros::basic_outcomes::SUCCEED;
  }

  void print_feedback(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard, std::shared_ptr<const GoToWaypoint::Feedback> feedback) {
    (void)blackboard;
    fprintf(stderr, "Current position: x = %f, y = %f, z = %f\n",
            feedback->current_pose.pose.position.x,
            feedback->current_pose.pose.position.y,
            feedback->current_pose.pose.position.z);
  }
};

class AbortState : public yasmin_ros::ActionState<GoToWaypoint> {
public:
  AbortState() : yasmin_ros::ActionState<GoToWaypoint>(
                     "/abort",
                     std::bind(&AbortState::create_goal_handler, this, _1),
                     std::bind(&AbortState::response_handler, this, _1, _2),
                     std::bind(&AbortState::print_feedback, this, _1, _2)) {};

  GoToWaypoint::Goal create_goal_handler(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    auto goal = GoToWaypoint::Goal();
    goal.waypoint = blackboard->get<PoseStamped>("home_pose");
    return goal;
  }

  std::string response_handler(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard, GoToWaypoint::Result::SharedPtr response) {
    (void)blackboard;
    (void)response;
    return yasmin_ros::basic_outcomes::SUCCEED;
  }

  void print_feedback(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard, std::shared_ptr<const GoToWaypoint::Feedback> feedback) {
    (void)blackboard;
    fprintf(stderr, "Current position: x = %f, y = %f, z = %f\n",
            feedback->current_pose.pose.position.x,
            feedback->current_pose.pose.position.y,
            feedback->current_pose.pose.position.z);
  }
};

std::string
ErrorState(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
  (void)blackboard;
  return yasmin_ros::basic_outcomes::CANCEL;
};

int main(int argc, char *argv[]) {

  YASMIN_LOG_INFO("docking");
  rclcpp::init(argc, argv);

  // set ROS 2 logs
  yasmin_ros::set_ros_loggers();

  // create a state machine
  auto sm = std::make_shared<yasmin::StateMachine>(
      std::initializer_list<std::string>{"outcome4",
                                         yasmin_ros::basic_outcomes::SUCCEED,
                                         yasmin_ros::basic_outcomes::CANCEL,
                                         yasmin_ros::basic_outcomes::ABORT});

  // cancel state machine on ROS 2 shutdown
  rclcpp::on_shutdown([sm]() {
    if (sm->is_running()) {
      sm->cancel_state();
    }
  });

  // add states
  sm->add_state("FIND_DOCK", std::make_shared<FindDockState>(),
                {
                    {yasmin_ros::basic_outcomes::SUCCEED, "GO_TO_DOCK"},
                    {yasmin_ros::basic_outcomes::CANCEL, "outcome4"},
                    {yasmin_ros::basic_outcomes::ABORT, "outcome4"},
                });
  sm->add_state("GO_TO_DOCK", std::make_shared<GoToDockState>(),
                {
                    {yasmin_ros::basic_outcomes::SUCCEED, "DOCK"},
                    {yasmin_ros::basic_outcomes::CANCEL, "outcome4"},
                    {yasmin_ros::basic_outcomes::ABORT, "ABORT"},
                });
  sm->add_state("DOCK", std::make_shared<DockState>(),
                {
                    {yasmin_ros::basic_outcomes::SUCCEED, "DOCKED"},
                    {yasmin_ros::basic_outcomes::CANCEL, "outcome4"},
                    {yasmin_ros::basic_outcomes::ABORT, "ABORT"},
                });
  sm->add_state("DOCKED", std::make_shared<yasmin::CbState>(std::initializer_list<std::string>{"outcome4", yasmin_ros::basic_outcomes::SUCCEED, yasmin_ros::basic_outcomes::CANCEL, yasmin_ros::basic_outcomes::ABORT}, DockedState),
                {
                    {yasmin_ros::basic_outcomes::SUCCEED, "RETURN_HOME"},
                    {yasmin_ros::basic_outcomes::CANCEL, "RETURN_HOME"},
                    {yasmin_ros::basic_outcomes::ABORT, "RETURN_HOME"},
                });
  sm->add_state("RETURN_HOME", std::make_shared<ReturnHomeState>(),
                {
                    {yasmin_ros::basic_outcomes::SUCCEED, "FIND_DOCK"},
                    {yasmin_ros::basic_outcomes::CANCEL, "outcome4"},
                    {yasmin_ros::basic_outcomes::ABORT, "ABORT"},
                });
  sm->add_state("ABORT", std::make_shared<AbortState>(),
                {
                    {yasmin_ros::basic_outcomes::SUCCEED, "outcome4"},
                    {yasmin_ros::basic_outcomes::CANCEL, "outcome4"},
                    {yasmin_ros::basic_outcomes::ABORT, "ABORT"},
                });
  sm->add_state("ERROR", std::make_shared<yasmin::CbState>(std::initializer_list<std::string>{"outcome4", yasmin_ros::basic_outcomes::SUCCEED, yasmin_ros::basic_outcomes::CANCEL, yasmin_ros::basic_outcomes::ABORT}, ErrorState),
                {
                    {yasmin_ros::basic_outcomes::SUCCEED, "outcome4"},
                    {yasmin_ros::basic_outcomes::CANCEL, "RETURN_HOME"},
                    {yasmin_ros::basic_outcomes::ABORT, "RETURN_HOME"},
                });

  // pub
  yasmin_viewer::YasminViewerPub yasmin_pub("YASMIN_ACTION_CLIENT_DEMO", sm);

  // create an initial blackboard
  std::shared_ptr<yasmin::blackboard::Blackboard> blackboard =
      std::make_shared<yasmin::blackboard::Blackboard>();
  blackboard->set<int>("n", 10);
  blackboard->set<bool>("start_search", true);
  blackboard->set<PoseStamped>("start_pose", PoseStamped());
  blackboard->set<float>("start_pose.pose.position.x", 0.0f);
  blackboard->set<float>("start_pose.pose.position.y", 0.0f);
  blackboard->set<float>("start_pose.pose.position.z", 0.0f);
  blackboard->set<PoseStamped>("dock_pose", PoseStamped());
  blackboard->set<float>("dock_pose.pose.position.x", 5.0);
  blackboard->set<float>("dock_pose.pose.position.y", 5.0);
  blackboard->set<float>("dock_pose.pose.position.z", 10.0);
  blackboard->set<bool>("return_home", true);

  // execute
  try {
    std::string outcome = (*sm.get())(blackboard);
    YASMIN_LOG_INFO(outcome.c_str());
  } catch (const std::exception &e) {
    YASMIN_LOG_WARN(e.what());
  }

  rclcpp::shutdown();

  return 0;
}
