
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
using GoToWaypoint = vortex_msgs::action::GoToWaypoint;
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
    return goal;
  }

  std::string response_handler(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard, FindDock::Result::SharedPtr response) {
    blackboard->set<PoseStamped>("pipeline_start_pose", response->dock_pose);
    return yasmin_ros::basic_outcomes::SUCCEED;
  }

  void print_feedback(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard, std::shared_ptr<const FindDock::Feedback> feedback) {
    (void)blackboard;
    fprintf(stderr, "Time elapsed: %f\n",
            feedback->time_elapsed);
  }
};

class GoToStartPipelineState : public yasmin_ros::ActionState<NavigateWaypoints> {
public:
  GoToStartPipelineState() : yasmin_ros::ActionState<NavigateWaypoints>(
                                 "/navigate_waypoints",
                                 std::bind(&GoToStartPipelineState::create_goal_handler, this, _1),
                                 std::bind(&GoToStartPipelineState::response_handler, this, _1, _2),
                                 std::bind(&GoToStartPipelineState::print_feedback, this, _1, _2)) {};

  NavigateWaypoints::Goal create_goal_handler(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    auto goal = NavigateWaypoints::Goal();

    std::vector<PoseStamped> waypoints;
    waypoints.push_back(blackboard->get<PoseStamped>("pipeline_start_pose"));
    // Print the pipeline start pose
    fprintf(stderr, "Pipeline start pose: x = %f, y = %f, z = %f\n",
            waypoints[0].pose.position.x,
            waypoints[0].pose.position.y,
            waypoints[0].pose.position.z);
    fprintf(stderr, "Pipeline start pose orientation: w = %f, x = %f, y = %f, z = %f\n",
            waypoints[0].pose.orientation.w,
            waypoints[0].pose.orientation.x,
            waypoints[0].pose.orientation.y,
            waypoints[0].pose.orientation.z);
    goal.waypoints = waypoints;
    return goal;
  }

  std::string response_handler(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard, NavigateWaypoints::Result::SharedPtr response) {
    (void)blackboard;
    if (response->success) {
      return yasmin_ros::basic_outcomes::SUCCEED;
    } else {
      return yasmin_ros::basic_outcomes::ABORT;
    }
  }

  void print_feedback(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard, std::shared_ptr<const NavigateWaypoints::Feedback> feedback) {
    blackboard->set<Pose>("current_pose", feedback->current_pose);
    fprintf(stderr, "Current position: x = %f, y = %f, z = %f\n",
            feedback->current_pose.position.x,
            feedback->current_pose.position.y,
            feedback->current_pose.position.z);
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

    return goal;
  }

  std::string response_handler(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard, NavigateWaypoints::Result::SharedPtr response) {
    if (response->success) {
      blackboard->set<bool>("is_home", true);
      return yasmin_ros::basic_outcomes::SUCCEED;
    } else {
      return yasmin_ros::basic_outcomes::ABORT;
    }
  }

  void print_feedback(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard, std::shared_ptr<const NavigateWaypoints::Feedback> feedback) {
    (void)blackboard;
    fprintf(stderr, "Current position: x = %f, y = %f, z = %f\n",
            feedback->current_pose.position.x,
            feedback->current_pose.position.y,
            feedback->current_pose.position.z);
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

std::string
ErrorState(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
  (void)blackboard;
  return yasmin_ros::basic_outcomes::ABORT;
};

class FollowPipelineState : public yasmin_ros::ActionState<NavigateWaypoints> {
public:
  FollowPipelineState() : yasmin_ros::ActionState<NavigateWaypoints>(
                                "/navigate_waypoints",
                                std::bind(&FollowPipelineState::create_goal_handler, this, _1),
                                std::bind(&FollowPipelineState::response_handler, this, _1, _2),
                                std::bind(&FollowPipelineState::print_feedback, this, _1, _2)) {};

  NavigateWaypoints::Goal create_goal_handler(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    auto goal = NavigateWaypoints::Goal();
    goal.waypoints.push_back(blackboard->get<PoseStamped>("aruco_waypoints"));
    return goal;
  }

  std::string response_handler(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard, NavigateWaypoints::Result::SharedPtr response) {
    (void)blackboard;
    if (response->success) {
      return yasmin_ros::basic_outcomes::SUCCEED;
    } else {
      return yasmin_ros::basic_outcomes::ABORT;
    }
  }

  void print_feedback(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard, std::shared_ptr<const NavigateWaypoints::Feedback> feedback) {
    blackboard->set<Pose>("current_pose", feedback->current_pose);
    fprintf(stderr, "Current position: x = %f, y = %f, z = %f\n",
            feedback->current_pose.position.x,
            feedback->current_pose.position.y,
            feedback->current_pose.position.z);
  }
};



int main(int argc, char *argv[]) {

  YASMIN_LOG_INFO("pipeline_demo_cpp");
  rclcpp::init(argc, argv);

  // set ROS 2 logs
  yasmin_ros::set_ros_loggers();

  // create a state machine
  auto sm = std::make_shared<yasmin::StateMachine>(
      std::initializer_list<std::string>{
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
  sm->add_state("FIND_PIPELINE", std::make_shared<FindPipelineState>(),
                {
                    {yasmin_ros::basic_outcomes::SUCCEED, "GO_TO_START_OF_PIPELINE"},
                    {yasmin_ros::basic_outcomes::CANCEL, yasmin_ros::basic_outcomes::CANCEL},
                    {yasmin_ros::basic_outcomes::ABORT, "ABORT"},
                });
  sm->add_state("GO_TO_START_OF_PIPELINE", std::make_shared<GoToStartPipelineState>(),
                {
                    {yasmin_ros::basic_outcomes::SUCCEED, "FOLLOW_PIPELINE"},
                    {yasmin_ros::basic_outcomes::CANCEL, yasmin_ros::basic_outcomes::CANCEL},
                    {yasmin_ros::basic_outcomes::ABORT, "ABORT"},
                });
  sm->add_state("RETURN_HOME", std::make_shared<ReturnHomeState>(),
                {
                    {yasmin_ros::basic_outcomes::SUCCEED, "FIND_PIPELINE"},
                    {yasmin_ros::basic_outcomes::CANCEL, yasmin_ros::basic_outcomes::CANCEL},
                    {yasmin_ros::basic_outcomes::ABORT, "ABORT"},
                });
  sm->add_state("ABORT", std::make_shared<AbortState>(),
                {
                    {yasmin_ros::basic_outcomes::ABORT, yasmin_ros::basic_outcomes::ABORT},
                });
  sm->add_state("ERROR", std::make_shared<yasmin::CbState>(std::initializer_list<std::string>{yasmin_ros::basic_outcomes::ABORT}, ErrorState),
                {
                    {yasmin_ros::basic_outcomes::ABORT, yasmin_ros::basic_outcomes::ABORT},
                });

  sm->add_state("FOLLOW_PIPELINE", std::make_shared<FollowPipelineState>(),
                       {
                           {yasmin_ros::basic_outcomes::SUCCEED, "RETURN_HOME"},
                           {yasmin_ros::basic_outcomes::ABORT, yasmin_ros::basic_outcomes::ABORT},
                       });



  // pub
  yasmin_viewer::YasminViewerPub yasmin_pub("Pipeline", sm);

  // create an initial blackboard
  std::shared_ptr<yasmin::blackboard::Blackboard> blackboard =
      std::make_shared<yasmin::blackboard::Blackboard>();

  blackboard->set<bool>("start_search", true);

  PoseStamped current_pose;
  current_pose.pose.position.x = 0.0;
  current_pose.pose.position.y = 0.0;
  current_pose.pose.position.z = 0.0;
  blackboard->set<PoseStamped>("current_pose", current_pose);
  blackboard->set<PoseStamped>("start_pose", current_pose);

  PoseStamped pipeline_start_pose;
  pipeline_start_pose.pose.position.x = 1.0;
  pipeline_start_pose.pose.position.y = 1.0;
  pipeline_start_pose.pose.position.z = 1.0;
  pipeline_start_pose.pose.orientation.w = 1.0;
  pipeline_start_pose.pose.orientation.x = 0.0;
  pipeline_start_pose.pose.orientation.y = 0.0;
  pipeline_start_pose.pose.orientation.z = 0.0;

  blackboard->set<PoseStamped>("pipeline_start_pose", pipeline_start_pose);
  
  std::vector<PoseStamped> aruco_waypoints;
  aruco_waypoints.push_back(PoseStamped());
  aruco_waypoints[0].pose.position.x = 1.0;
  aruco_waypoints[0].pose.position.y = 1.0;
  aruco_waypoints[0].pose.position.z = 1.0;
  
  blackboard->set<std::vector<PoseStamped>>("aruco_waypoints", aruco_waypoints);

  blackboard->set<bool>("return_home", true);
  blackboard->set<int>("num_remaining_codes", 5);

  // execute
  try {
    std::string outcome = (*sm.get())(blackboard);
    YASMIN_LOG_INFO(outcome.c_str());
  } catch (const std::exception &e) {
    YASMIN_LOG_WARN(e.what());
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
