#include <memory>
#include <string>

#include "pipeline_fsm/mock_states.hpp"
#include "pipeline_fsm/waypoint_manager_states.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_ros/ros_logs.hpp"
#include "yasmin_ros/yasmin_node.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

class MissionExecutionControllerNode : public rclcpp::Node {
public:
  MissionExecutionControllerNode()
      : Node("mission_execution_controller"),
        fsm_running_(false),
        active_outer_state_("IDLE"),
        active_inner_state_("") {

    blackboard_ = yasmin::Blackboard::make_shared();
    fsm_ = create_mission_fsm();

    start_service_ = this->create_service<std_srvs::srv::Trigger>(
        "mission/start",
        std::bind(&MissionExecutionControllerNode::handle_start, this,
                  std::placeholders::_1, std::placeholders::_2));

    pause_service_ = this->create_service<std_srvs::srv::Trigger>(
        "mission/pause",
        std::bind(&MissionExecutionControllerNode::handle_pause, this,
                  std::placeholders::_1, std::placeholders::_2));

    resume_service_ = this->create_service<std_srvs::srv::Trigger>(
        "mission/resume",
        std::bind(&MissionExecutionControllerNode::handle_resume, this,
                  std::placeholders::_1, std::placeholders::_2));

    stop_service_ = this->create_service<std_srvs::srv::Trigger>(
        "mission/stop",
        std::bind(&MissionExecutionControllerNode::handle_stop, this,
                  std::placeholders::_1, std::placeholders::_2));

    viewer_ = std::make_unique<yasmin_viewer::YasminViewerPub>(
        fsm_, "PIPELINE_MISSION");

    YASMIN_LOG_INFO("MissionExecutionControllerNode ready");
  }

private:
  yasmin::StateMachine::SharedPtr create_mission_fsm() {
    auto sm = yasmin::StateMachine::make_shared(
        std::initializer_list<std::string>{"done", "aborted"}, false);

    sm->add_state("IDLE", std::make_shared<pipeline_fsm::IdleState>(),
                  {{yasmin_ros::basic_outcomes::SUCCEED, "SEARCH"}});

    sm->add_state("SEARCH", create_search_superstate(),
                  {{yasmin_ros::basic_outcomes::SUCCEED, "CONVERGE"},
                   {yasmin_ros::basic_outcomes::ABORT, "ABORTED"}});

    sm->add_state("CONVERGE", std::make_shared<pipeline_fsm::MockConvergeState>(),
                  {{yasmin_ros::basic_outcomes::SUCCEED, "PIPELINE_FOLLOWING"},
                   {yasmin_ros::basic_outcomes::ABORT, "ABORTED"}});

    sm->add_state("PIPELINE_FOLLOWING", create_pipeline_following_superstate(),
                  {{yasmin_ros::basic_outcomes::SUCCEED, "DONE"},
                   {yasmin_ros::basic_outcomes::ABORT, "ABORTED"}});

    sm->add_state("DONE", std::make_shared<pipeline_fsm::DoneState>(),
                  {{"done", "done"}});

    sm->add_state("ABORTED", std::make_shared<pipeline_fsm::AbortedState>(),
                  {{"aborted", "aborted"}});

    sm->add_start_cb([this](yasmin::Blackboard::SharedPtr, const std::string &start_state) {
      active_outer_state_ = start_state;
      // Clear inner state when entering a new outer state
      if (start_state != "SEARCH" && start_state != "PIPELINE_FOLLOWING") {
        active_inner_state_ = "";
      }
      YASMIN_LOG_INFO(("FSM started at: " + start_state).c_str());
    });

    sm->add_transition_cb(
        [this](yasmin::Blackboard::SharedPtr, const std::string &from,
               const std::string &to, const std::string &outcome) {
          active_outer_state_ = to;
          // Clear inner state when transitioning to a non-nested state
          if (to != "SEARCH" && to != "PIPELINE_FOLLOWING") {
            active_inner_state_ = "";
          }
          YASMIN_LOG_INFO(
              ("Transition: " + from + " -> " + to + " (" + outcome + ")")
                  .c_str());
        });

    return sm;
  }

  yasmin::StateMachine::SharedPtr create_search_superstate() {
    search_sm_ = yasmin::StateMachine::make_shared(
        std::initializer_list<std::string>{yasmin_ros::basic_outcomes::SUCCEED,
                                           yasmin_ros::basic_outcomes::ABORT});

    search_sm_->add_state(
        "START_WAYPOINT_MANAGER",
        std::make_shared<pipeline_fsm::StartWaypointManagerState>(),
        {{yasmin_ros::basic_outcomes::SUCCEED, "SEARCH_PATTERN"},
         {yasmin_ros::basic_outcomes::ABORT, yasmin_ros::basic_outcomes::ABORT}});

    search_sm_->add_state(
        "SEARCH_PATTERN",
        std::make_shared<pipeline_fsm::MockSearchPatternState>(),
        {{yasmin_ros::basic_outcomes::SUCCEED, "STOP_WAYPOINT_MANAGER"},
         {yasmin_ros::basic_outcomes::ABORT, yasmin_ros::basic_outcomes::ABORT}});

    search_sm_->add_state(
        "STOP_WAYPOINT_MANAGER",
        std::make_shared<pipeline_fsm::StopWaypointManagerState>(),
        {{yasmin_ros::basic_outcomes::SUCCEED,
          yasmin_ros::basic_outcomes::SUCCEED}});

    // Track inner state transitions for SEARCH
    search_sm_->add_start_cb([this](yasmin::Blackboard::SharedPtr, const std::string &start_state) {
      if (active_outer_state_ == "SEARCH") {
        active_inner_state_ = start_state;
        YASMIN_LOG_INFO(("SEARCH inner state: " + start_state).c_str());
      }
    });

    search_sm_->add_transition_cb(
        [this](yasmin::Blackboard::SharedPtr, const std::string &from,
               const std::string &to, const std::string &outcome) {
          if (active_outer_state_ == "SEARCH") {
            active_inner_state_ = to;
            YASMIN_LOG_INFO(("SEARCH transition: " + from + " -> " + to).c_str());
          }
        });

    return search_sm_;
  }

  yasmin::StateMachine::SharedPtr create_pipeline_following_superstate() {
    pipeline_sm_ = yasmin::StateMachine::make_shared(
        std::initializer_list<std::string>{yasmin_ros::basic_outcomes::SUCCEED,
                                           yasmin_ros::basic_outcomes::ABORT});

    pipeline_sm_->add_state(
        "START_WAYPOINT_MANAGER",
        std::make_shared<pipeline_fsm::StartWaypointManagerState>(),
        {{yasmin_ros::basic_outcomes::SUCCEED, "PIPELINE_FOLLOW_ACTION"},
         {yasmin_ros::basic_outcomes::ABORT, yasmin_ros::basic_outcomes::ABORT}});

    pipeline_sm_->add_state(
        "PIPELINE_FOLLOW_ACTION",
        std::make_shared<pipeline_fsm::MockPipelineFollowState>(),
        {{yasmin_ros::basic_outcomes::SUCCEED, "STOP_WAYPOINT_MANAGER"},
         {yasmin_ros::basic_outcomes::ABORT, yasmin_ros::basic_outcomes::ABORT}});

    pipeline_sm_->add_state(
        "STOP_WAYPOINT_MANAGER",
        std::make_shared<pipeline_fsm::StopWaypointManagerState>(),
        {{yasmin_ros::basic_outcomes::SUCCEED,
          yasmin_ros::basic_outcomes::SUCCEED}});

    // Track inner state transitions for PIPELINE_FOLLOWING
    pipeline_sm_->add_start_cb([this](yasmin::Blackboard::SharedPtr, const std::string &start_state) {
      if (active_outer_state_ == "PIPELINE_FOLLOWING") {
        active_inner_state_ = start_state;
        YASMIN_LOG_INFO(("PIPELINE_FOLLOWING inner state: " + start_state).c_str());
      }
    });

    pipeline_sm_->add_transition_cb(
        [this](yasmin::Blackboard::SharedPtr, const std::string &from,
               const std::string &to, const std::string &outcome) {
          if (active_outer_state_ == "PIPELINE_FOLLOWING") {
            active_inner_state_ = to;
            YASMIN_LOG_INFO(("PIPELINE_FOLLOWING transition: " + from + " -> " + to).c_str());
          }
        });

    return pipeline_sm_;
  }

  void handle_start(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    if (fsm_running_) {
      res->success = false;
      res->message = "Mission already running";
      return;
    }

    YASMIN_LOG_INFO("Starting mission");
    fsm_running_ = true;
    active_outer_state_ = "IDLE";
    active_inner_state_ = "";  // Clear inner state on start

    execution_thread_ = std::thread([this]() {
      try {
        std::string outcome = (*fsm_.get())(blackboard_);
        YASMIN_LOG_INFO(("Mission outcome: " + outcome).c_str());
      } catch (const std::exception &e) {
        YASMIN_LOG_WARN(("Mission exception: " + std::string(e.what())).c_str());
      }
      fsm_running_ = false;
    });

    execution_thread_.detach();

    res->success = true;
    res->message = "Mission started";
  }

  void handle_pause(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    if (!fsm_running_) {
      res->success = false;
      res->message = "Mission not running";
      return;
    }

    // Save BOTH outer and inner states BEFORE cancellation
    std::string paused_outer_state = active_outer_state_;
    std::string paused_inner_state = active_inner_state_;
    
    std::string pause_msg = "Pausing mission at: " + paused_outer_state;
    if (!paused_inner_state.empty()) {
      pause_msg += " -> " + paused_inner_state;
    }
    YASMIN_LOG_INFO(pause_msg.c_str());

    fsm_->cancel_state();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    fsm_running_ = false;
    
    // Restore the states we paused at (cancel might have transitioned to ABORTED)
    active_outer_state_ = paused_outer_state;
    active_inner_state_ = paused_inner_state;

    res->success = true;
    res->message = pause_msg;
  }

  void handle_resume(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    if (fsm_running_) {
      res->success = false;
      res->message = "Mission already running";
      return;
    }

    std::string resume_msg = "Resuming mission from: " + active_outer_state_;
    if (!active_inner_state_.empty()) {
      resume_msg += " -> " + active_inner_state_;
    }
    YASMIN_LOG_INFO(resume_msg.c_str());

    // Set outer state
    fsm_->set_start_state(active_outer_state_);
    
    // Set inner state if we're resuming a nested FSM
    if (!active_inner_state_.empty()) {
      if (active_outer_state_ == "SEARCH" && search_sm_) {
        search_sm_->set_start_state(active_inner_state_);
      } else if (active_outer_state_ == "PIPELINE_FOLLOWING" && pipeline_sm_) {
        pipeline_sm_->set_start_state(active_inner_state_);
      }
    }
    
    fsm_running_ = true;

    execution_thread_ = std::thread([this]() {
      try {
        std::string outcome = (*fsm_.get())(blackboard_);
        YASMIN_LOG_INFO(("Mission outcome: " + outcome).c_str());
      } catch (const std::exception &e) {
        YASMIN_LOG_WARN(("Mission exception: " + std::string(e.what())).c_str());
      }
      fsm_running_ = false;
    });

    execution_thread_.detach();

    res->success = true;
    res->message = resume_msg;
  }

  void handle_stop(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    if (!fsm_running_) {
      res->success = false;
      res->message = "Mission not running";
      return;
    }

    YASMIN_LOG_INFO("Stopping mission");

    fsm_->cancel_state();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    fsm_running_ = false;
    active_outer_state_ = "IDLE";
    active_inner_state_ = "";  // Clear inner state on stop

    res->success = true;
    res->message = "Mission stopped";
  }

  yasmin::StateMachine::SharedPtr fsm_;
  yasmin::StateMachine::SharedPtr search_sm_;
  yasmin::StateMachine::SharedPtr pipeline_sm_;
  yasmin::Blackboard::SharedPtr blackboard_;
  std::unique_ptr<yasmin_viewer::YasminViewerPub> viewer_;

  std::atomic<bool> fsm_running_;
  std::string active_outer_state_;
  std::string active_inner_state_;  // Track inner state of nested FSMs
  std::thread execution_thread_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  yasmin_ros::set_ros_loggers();

  auto node = std::make_shared<MissionExecutionControllerNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
