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
  FindPipelineState() : yasmin_ros::ActionState<FindDock>(
                            "/find_pipeline",
                            std::bind(&FindPipelineState::create_goal_handler, this, _1),
                            std::bind(&FindPipelineState::response_handler, this, _1, _2),
                            std::bind(&FindPipelineState::print_feedback, this, _1, _2)) {};

  FindDock::Goal create_goal_handler(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    auto goal = FindDock::Goal();
    goal.start_search = blackboard->get<bool>("start_search");

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal sent to action server: ");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  Start search: %s",
                goal.start_search ? "true" : "false");
    return goal;
  }

  std::string response_handler(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard, FindDock::Result::SharedPtr response) {
    blackboard->set<PoseStamped>("pipeline_start_pose", response->dock_pose);
    // If found position is (0, 0, 0), the pipeline was not found
    if (response->dock_pose.pose.position.x == 0.0 &&
        response->dock_pose.pose.position.y == 0.0 &&
        response->dock_pose.pose.position.z == 0.0) {
      return yasmin_ros::basic_outcomes::ABORT;
    } else {
      return yasmin_ros::basic_outcomes::SUCCEED;
    }
    
  }

  void print_feedback(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard, std::shared_ptr<const FindDock::Feedback> feedback) {
    blackboard->set<float>("time_elapsed", feedback->time_elapsed);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Time elapsed: %f",
                feedback->time_elapsed);
  }
};

class ReturnHomeState : public yasmin_ros::ActionState<NavigateWaypoints> {
public:
  ReturnHomeState() : yasmin_ros::ActionState<NavigateWaypoints>(
                          "/navigate_waypoints",
                          std::bind(&ReturnHomeState::create_goal_handler, this, _1),
                          std::bind(&ReturnHomeState::response_handler, this, _1, _2),
                          std::bind(&ReturnHomeState::print_feedback, this, _1, _2)) {};

  NavigateWaypoints::Goal create_goal_handler(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    auto goal = NavigateWaypoints::Goal();
    goal.waypoints.push_back(blackboard->get<PoseStamped>("start_pose"));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal sent to action server: ");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Returning home to position x = %f, y = %f, z = %f",
                goal.waypoints.at(0).pose.position.x,
                goal.waypoints.at(0).pose.position.y,
                goal.waypoints.at(0).pose.position.z);

    return goal;
  }

  std::string response_handler(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard, NavigateWaypoints::Result::SharedPtr response) {
      blackboard->set<bool>("is_home", response->success);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Response received from action server: ");
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  Success: %s",
                  response->success ? "true" : "false");
      if (response->success) {
        return yasmin_ros::basic_outcomes::SUCCEED;
    } else {
        return yasmin_ros::basic_outcomes::ABORT;
    }
  }

  void print_feedback(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard, std::shared_ptr<const NavigateWaypoints::Feedback> feedback) {
    blackboard->set<Pose>("current_pose", feedback->current_pose);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Current position: x = %f, y = %f, z = %f",
                feedback->current_pose.position.x,
                feedback->current_pose.position.y,
                feedback->current_pose.position.z); 
  }
};

// Abort State 
std::string AbortState(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
  (void)blackboard;
  return yasmin_ros::basic_outcomes::ABORT;
};


std::string
ErrorState(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
  (void)blackboard;
  return yasmin_ros::basic_outcomes::ABORT;
};

std::shared_ptr<yasmin::StateMachine> create_state_machine() {
  auto sm = std::make_shared<yasmin::StateMachine>(
      std::initializer_list<std::string>{
                                         yasmin_ros::basic_outcomes::SUCCEED,
                                         yasmin_ros::basic_outcomes::ABORT, 
                                         "error"});
  return sm;
}

void add_states(std::shared_ptr<yasmin::StateMachine> sm) {
  sm->add_state("FIND_VALVE", std::make_shared<FindPipelineState>(),
                {
                    {yasmin_ros::basic_outcomes::SUCCEED, "GO_TO_VALVE"},
                    {yasmin_ros::basic_outcomes::CANCEL, "ERROR"},
                    {yasmin_ros::basic_outcomes::ABORT, "ABORT"},
                });
  sm->add_state("GO_TO_VALVE", std::make_shared<ReturnHomeState>(),
                {
                    {yasmin_ros::basic_outcomes::SUCCEED, "TURN_VALVE"},
                    {yasmin_ros::basic_outcomes::CANCEL, "ERROR"},
                    {yasmin_ros::basic_outcomes::ABORT, "ABORT"},
                });
  sm->add_state("TURN_VALVE", std::make_shared<ReturnHomeState>(),
                {
                    {yasmin_ros::basic_outcomes::SUCCEED, "MOVE_AWAY_FROM_VALVE"},
                    {yasmin_ros::basic_outcomes::CANCEL, "ERROR"},
                    {yasmin_ros::basic_outcomes::ABORT, "ABORT"},
                });
  sm->add_state("MOVE_AWAY_FROM_VALVE", std::make_shared<ReturnHomeState>(),
                {
                    {yasmin_ros::basic_outcomes::SUCCEED, yasmin_ros::basic_outcomes::SUCCEED},
                    {yasmin_ros::basic_outcomes::CANCEL, "GO_TO_VALVE"},
                    {yasmin_ros::basic_outcomes::ABORT, "ABORT"},
                });
  sm->add_state("ABORT", std::make_shared<yasmin::CbState>(std::initializer_list<std::string>{yasmin_ros::basic_outcomes::ABORT}, AbortState),
                {
                    {yasmin_ros::basic_outcomes::ABORT, yasmin_ros::basic_outcomes::ABORT},
                });
  sm->add_state("ERROR", std::make_shared<yasmin::CbState>(std::initializer_list<std::string>{yasmin_ros::basic_outcomes::ABORT}, ErrorState),
                {
                    {yasmin_ros::basic_outcomes::ABORT, "error"},
                });
}

int main(int argc, char *argv[]) {
  YASMIN_LOG_INFO("structure_cpp");
  rclcpp::init(argc, argv);

  // set ROS 2 logs
  yasmin_ros::set_ros_loggers();

  // create a state machine
  auto sm = create_state_machine();

  // cancel state machine on ROS 2 shutdown
  rclcpp::on_shutdown([sm]() {
    if (sm->is_running()) {
      sm->cancel_state();
    }
  });

  // add states
  add_states(sm);

  // pub
  yasmin_viewer::YasminViewerPub yasmin_pub("Structure", sm);

  // create an initial blackboard
  std::shared_ptr<yasmin::blackboard::Blackboard> blackboard =
      std::make_shared<yasmin::blackboard::Blackboard>();

  blackboard->set<bool>("start_search", true);
  blackboard->set<float>("time_elapsed", 0.0);

  PoseStamped current_pose;
  current_pose.pose.position.x = 0.0;
  current_pose.pose.position.y = 0.0;
  current_pose.pose.position.z = 0.0;
  blackboard->set<PoseStamped>("current_pose", current_pose);
  blackboard->set<PoseStamped>("start_pose", current_pose);

  PoseStamped valve_pose;
  valve_pose.pose.position.x = 1.0;
  valve_pose.pose.position.y = 1.0;
  valve_pose.pose.position.z = 1.0;
  valve_pose.pose.orientation.w = 1.0;
  blackboard->set<PoseStamped>("valve_pose", valve_pose);
  
  
  blackboard->set<bool>("return_home", true);
  blackboard->set<int>("num_remaining_attempts", 5);

  // execute
  try {
    std::string outcome = (*sm.get())(blackboard);
    YASMIN_LOG_INFO(outcome.c_str());
  } catch (const std::exception &e) {
    YASMIN_LOG_WARN(e.what());
    rcutils_reset_error();
  }

  if (rclcpp::ok()) {
        // Explicitly reset state machine resources if needed
        sm.reset();
        blackboard.reset();

        // Shut down ROS2 context
        rclcpp::shutdown();

        YASMIN_LOG_INFO("ROS2 shutdown completed.");
  }
  
  return 0;
}
