
#include <chrono>
#include <iostream>
#include <memory>
#include <string>

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
using FindDock = vortex_msgs::action::FindDock;
using GoToWaypoint = vortex_msgs::action::GoToWaypoint;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using namespace yasmin;

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

class GoOverDock : public yasmin_ros::ActionState<GoToWaypoint> {
public:
  GoOverDock() : yasmin_ros::ActionState<GoToWaypoint>(
                     "/go_right_over",
                     std::bind(&GoOverDock::create_goal_handler, this, _1),
                     std::bind(&GoOverDock::response_handler, this, _1, _2),
                     std::bind(&GoOverDock::print_feedback, this, _1, _2)) {};

  GoToWaypoint::Goal create_goal_handler(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    auto goal = GoToWaypoint::Goal();
    goal.waypoint = blackboard->get<PoseStamped>("dock_pose");
    goal.waypoint.pose.position.x += 1.0;
    return goal;
  }

  std::string response_handler(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard, GoToWaypoint::Result::SharedPtr response) {
    (void)blackboard;
    (void)response;

    blackboard->set<bool>("has_finished_converging", response->success);
    if (blackboard->get<bool>("has_finished_converging")) {
      return yasmin_ros::basic_outcomes::SUCCEED;
    } else {
      return yasmin_ros::basic_outcomes::ABORT;
    }
  }

  void print_feedback(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard, std::shared_ptr<const GoToWaypoint::Feedback> feedback) {
    (void)blackboard;
    blackboard->set<PoseStamped>("current_pose", feedback->current_pose);
    fprintf(stderr, "Current position: x = %f, y = %f, z = %f\n",
            feedback->current_pose.pose.position.x,
            feedback->current_pose.pose.position.y,
            feedback->current_pose.pose.position.z);
  }
};

class GoDownDock : public yasmin_ros::ActionState<GoToWaypoint> {
public:
  GoDownDock() : yasmin_ros::ActionState<GoToWaypoint>(
                     "/go_down",
                     std::bind(&GoDownDock::create_goal_handler, this, _1),
                     std::bind(&GoDownDock::response_handler, this, _1, _2),
                     std::bind(&GoDownDock::print_feedback, this, _1, _2)) {};

  GoToWaypoint::Goal create_goal_handler(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    auto goal = GoToWaypoint::Goal();
    goal.waypoint = blackboard->get<PoseStamped>("docking_station_location");
    goal.waypoint.pose.position.z -= 1.0;
    return goal;
  }

  std::string response_handler(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard, GoToWaypoint::Result::SharedPtr response) {
    (void)blackboard;
    (void)response;

    blackboard->set<bool>("has_finished_converging", response->success);
    if (blackboard->get<bool>("has_finished_converging")) {
      return yasmin_ros::basic_outcomes::SUCCEED;
    } else {
      return yasmin_ros::basic_outcomes::ABORT;
    }
  }

  void print_feedback(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard, std::shared_ptr<const GoToWaypoint::Feedback> feedback) {
    (void)blackboard;
    blackboard->set<PoseStamped>("current_pose", feedback->current_pose);
    fprintf(stderr, "Current position: x = %f, y = %f, z = %f\n",
            feedback->current_pose.pose.position.x,
            feedback->current_pose.pose.position.y,
            feedback->current_pose.pose.position.z);
  }
};

std::string has_finished_converging(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
  (void)blackboard;
  return yasmin_ros::basic_outcomes::SUCCEED;
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

  auto nested_sm = std::make_shared<yasmin::StateMachine>(
      std::initializer_list<std::string>{
          "aborted",
          "finished",
      });

  // cancel state machine on ROS 2 shutdown
  rclcpp::on_shutdown([sm]() {
    if (sm->is_running()) {
      sm->cancel_state();
    }
  });

  rclcpp::on_shutdown([nested_sm]() {
    if (nested_sm->is_running()) {
      nested_sm->cancel_state();
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
                    {yasmin_ros::basic_outcomes::SUCCEED, "dock_fsm"},
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
                    {yasmin_ros::basic_outcomes::SUCCEED, "aborted"},
                    {yasmin_ros::basic_outcomes::CANCEL, "aborted"},
                    {yasmin_ros::basic_outcomes::ABORT, "aborted"},
                });
  sm->add_state("ERROR", std::make_shared<yasmin::CbState>(std::initializer_list<std::string>{"outcome4", yasmin_ros::basic_outcomes::SUCCEED, yasmin_ros::basic_outcomes::CANCEL, yasmin_ros::basic_outcomes::ABORT}, ErrorState),
                {
                    {yasmin_ros::basic_outcomes::SUCCEED, "outcome4"},
                    {yasmin_ros::basic_outcomes::CANCEL, "RETURN_HOME"},
                    {yasmin_ros::basic_outcomes::ABORT, "RETURN_HOME"},
                });

  nested_sm->add_state("GO_OVER_DOCK", std::make_shared<GoOverDock>(),
                       {
                           {yasmin_ros::basic_outcomes::SUCCEED, "GO_DOWN_DOCK"},
                           {yasmin_ros::basic_outcomes::ABORT, "aborted"},
                       });

  nested_sm->add_state("GO_DOWN_DOCK", std::make_shared<GoDownDock>(),
                       {
                           {yasmin_ros::basic_outcomes::SUCCEED, "has_finished_converging"},
                           {yasmin_ros::basic_outcomes::ABORT, "aborted"},
                       });
  nested_sm->add_state("has_finished_converging", std::make_shared<yasmin::CbState>(std::initializer_list<std::string>{yasmin_ros::basic_outcomes::SUCCEED, yasmin_ros::basic_outcomes::ABORT}, has_finished_converging),
                       {
                           {yasmin_ros::basic_outcomes::SUCCEED, "finished"},
                           {yasmin_ros::basic_outcomes::ABORT, "aborted"},
                       });

  sm->add_state("dock_fsm", nested_sm,
                {
                    {"finished", "DOCKED"},
                    {"aborted", "ABORT"},
                });

  // pub
  yasmin_viewer::YasminViewerPub yasmin_pub("Docking", sm);
  yasmin_viewer::YasminViewerPub yasmin_pub_nested("DockingNested", nested_sm);

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

/*
#include <iostream>
#include <memory>
#include <string>
#include <chrono>
#include <thread>

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

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using std::placeholders::_1;
using std::placeholders::_2;
using FindDock = vortex_msgs::action::FindDock;
using GoToWaypoint = vortex_msgs::action::GoToWaypoint;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using namespace yasmin;
using namespace std::chrono_literals;

class DockingNode : public rclcpp::Node {
public:
  DockingNode(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard)
      : Node("docking_node"), blackboard_(blackboard) {
    docking_state_pub_ = this->create_publisher<std_msgs::msg::String>("/docking_state", 10);
    timer_ = this->create_wall_timer(1s, std::bind(&DockingNode::timer_callback, this));
  }

private:
  void timer_callback() {
    auto message = std_msgs::msg::String();

    // Get the current state from the blackboard
    message.data = blackboard_->get<std::string>("current_state");


    docking_state_pub_->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr docking_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<yasmin::blackboard::Blackboard> blackboard_;
};

// Your other state classes (FindDockState, GoToDockState, etc.)
// ...
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

class GoOverDock : public yasmin_ros::ActionState<GoToWaypoint> {
public:
  GoOverDock() : yasmin_ros::ActionState<GoToWaypoint>(
                     "/go_right_over",
                     std::bind(&GoOverDock::create_goal_handler, this, _1),
                     std::bind(&GoOverDock::response_handler, this, _1, _2),
                     std::bind(&GoOverDock::print_feedback, this, _1, _2)) {};

  GoToWaypoint::Goal create_goal_handler(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    auto goal = GoToWaypoint::Goal();
    goal.waypoint = blackboard->get<PoseStamped>("dock_pose");
    goal.waypoint.pose.position.x += 1.0;
    return goal;
  }

  std::string response_handler(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard, GoToWaypoint::Result::SharedPtr response) {
    (void)blackboard;
    (void)response;

    blackboard->set<bool>("has_finished_converging", response->success);
    if (blackboard->get<bool>("has_finished_converging")) {
      return yasmin_ros::basic_outcomes::SUCCEED;
    } else {
      return yasmin_ros::basic_outcomes::ABORT;
    }
  }

  void print_feedback(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard, std::shared_ptr<const GoToWaypoint::Feedback> feedback) {
    (void)blackboard;
    blackboard->set<PoseStamped>("current_pose", feedback->current_pose);
    fprintf(stderr, "Current position: x = %f, y = %f, z = %f\n",
            feedback->current_pose.pose.position.x,
            feedback->current_pose.pose.position.y,
            feedback->current_pose.pose.position.z);
  }
};

class GoDownDock : public yasmin_ros::ActionState<GoToWaypoint> {
public:
  GoDownDock() : yasmin_ros::ActionState<GoToWaypoint>(
                     "/go_down",
                     std::bind(&GoDownDock::create_goal_handler, this, _1),
                     std::bind(&GoDownDock::response_handler, this, _1, _2),
                     std::bind(&GoDownDock::print_feedback, this, _1, _2)) {};

  GoToWaypoint::Goal create_goal_handler(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    auto goal = GoToWaypoint::Goal();
    goal.waypoint = blackboard->get<PoseStamped>("docking_station_location");
    goal.waypoint.pose.position.z -= 1.0;
    return goal;
  }

  std::string response_handler(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard, GoToWaypoint::Result::SharedPtr response) {
    (void)blackboard;
    (void)response;

    blackboard->set<bool>("has_finished_converging", response->success);
    if (blackboard->get<bool>("has_finished_converging")) {
      return yasmin_ros::basic_outcomes::SUCCEED;
    } else {
      return yasmin_ros::basic_outcomes::ABORT;
    }
  }

  void print_feedback(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard, std::shared_ptr<const GoToWaypoint::Feedback> feedback) {
    (void)blackboard;
    blackboard->set<PoseStamped>("current_pose", feedback->current_pose);
    fprintf(stderr, "Current position: x = %f, y = %f, z = %f\n",
            feedback->current_pose.pose.position.x,
            feedback->current_pose.pose.position.y,
            feedback->current_pose.pose.position.z);
  }
};

std::string has_finished_converging(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
  (void)blackboard;
  return yasmin_ros::basic_outcomes::SUCCEED;
};


int main(int argc, char *argv[]) {
  YASMIN_LOG_INFO("Starting docking node and state machine");
  rclcpp::init(argc, argv);

  // Set ROS 2 loggers
  yasmin_ros::set_ros_loggers();

  // Create an initial blackboard
  auto blackboard = std::make_shared<yasmin::blackboard::Blackboard>();
  blackboard->set<std::string>("current_state", "Initializing");

  // Create state machine and populate states as in your original code
  auto sm = std::make_shared<yasmin::StateMachine>(
      std::initializer_list<std::string>{"outcome4",
                                         yasmin_ros::basic_outcomes::SUCCEED,
                                         yasmin_ros::basic_outcomes::CANCEL,
                                         yasmin_ros::basic_outcomes::ABORT});

  auto nested_sm = std::make_shared<yasmin::StateMachine>(
      std::initializer_list<std::string>{"outcome4",
                                         yasmin_ros::basic_outcomes::SUCCEED,
                                         yasmin_ros::basic_outcomes::CANCEL,
                                         yasmin_ros::basic_outcomes::ABORT});

  // Define states and transitions
  sm->add_state("FIND_DOCK", std::make_shared<FindDockState>(),
                {
                    {yasmin_ros::basic_outcomes::SUCCEED, "GO_TO_DOCK"},
                    {yasmin_ros::basic_outcomes::CANCEL, "outcome4"},
                    {yasmin_ros::basic_outcomes::ABORT, "outcome4"},
                });

  // Add other states as in your code...
  sm->add_state("GO_TO_DOCK", std::make_shared<GoToDockState>(),
                {
                    {yasmin_ros::basic_outcomes::SUCCEED, "DOCK"},
                    {yasmin_ros::basic_outcomes::CANCEL, "outcome4"},
                    {yasmin_ros::basic_outcomes::ABORT, "ABORT"},
                });
  sm->add_state("DOCK", std::make_shared<DockState>(),
                {
                    {yasmin_ros::basic_outcomes::SUCCEED, "dock_fsm"},
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
                    {yasmin_ros::basic_outcomes::SUCCEED, "aborted"},
                    {yasmin_ros::basic_outcomes::CANCEL, "aborted"},
                    {yasmin_ros::basic_outcomes::ABORT, "aborted"},
                });
  sm->add_state("ERROR", std::make_shared<yasmin::CbState>(std::initializer_list<std::string>{"outcome4", yasmin_ros::basic_outcomes::SUCCEED, yasmin_ros::basic_outcomes::CANCEL, yasmin_ros::basic_outcomes::ABORT}, ErrorState),
                {
                    {yasmin_ros::basic_outcomes::SUCCEED, "outcome4"},
                    {yasmin_ros::basic_outcomes::CANCEL, "RETURN_HOME"},
                    {yasmin_ros::basic_outcomes::ABORT, "RETURN_HOME"},
                });

  nested_sm->add_state("GO_OVER_DOCK", std::make_shared<GoOverDock>(),
                       {
                           {yasmin_ros::basic_outcomes::SUCCEED, "GO_DOWN_DOCK"},
                           {yasmin_ros::basic_outcomes::ABORT, "aborted"},
                       });

  nested_sm->add_state("GO_DOWN_DOCK", std::make_shared<GoDownDock>(),
                       {
                           {yasmin_ros::basic_outcomes::SUCCEED, "has_finished_converging"},
                           {yasmin_ros::basic_outcomes::ABORT, "aborted"},
                       });
  nested_sm->add_state("has_finished_converging", std::make_shared<yasmin::CbState>(std::initializer_list<std::string>{yasmin_ros::basic_outcomes::SUCCEED, yasmin_ros::basic_outcomes::ABORT}, has_finished_converging),
                       {
                           {yasmin_ros::basic_outcomes::SUCCEED, "finished"},
                           {yasmin_ros::basic_outcomes::ABORT, "aborted"},
                       });

  sm->add_state("dock_fsm", nested_sm,
                {
                    {"finished", "DOCKED"},
                    {"aborted", "ABORT"},
                });

  // Create the DockingNode instance
  auto docking_node = std::make_shared<DockingNode>(blackboard);

  // Thread to spin the DockingNode
  std::thread node_thread([&]() {
    rclcpp::spin(docking_node);
  });

  // Thread to execute the state machine
  std::thread sm_thread([&]() {
    // Execute the state machine and update blackboard with current state
    try {
      while (rclcpp::ok()) {
        blackboard->set<std::string>("current_state", (*sm.get())(blackboard));
        rclcpp::sleep_for(100ms);  // Adjust frequency as needed
      }
    } catch (const std::exception &e) {
      YASMIN_LOG_WARN(e.what());
    }
  });

  // Wait for threads to complete
  node_thread.join();
  sm_thread.join();

  rclcpp::shutdown();
  return 0;
}
*/
