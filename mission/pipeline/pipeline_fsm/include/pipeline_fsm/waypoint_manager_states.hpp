#ifndef PIPELINE_FSM__WAYPOINT_MANAGER_STATES_HPP_
#define PIPELINE_FSM__WAYPOINT_MANAGER_STATES_HPP_

#include "rclcpp_action/rclcpp_action.hpp"
#include "vortex_msgs/action/waypoint_manager.hpp"
#include "yasmin/blackboard.hpp"
#include "yasmin/state.hpp"
#include "yasmin_ros/basic_outcomes.hpp"

namespace pipeline_fsm {

class StartWaypointManagerState : public yasmin::State {
public:
  StartWaypointManagerState();
  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;
  void cancel_state() override;

private:
  using WaypointManagerAction = vortex_msgs::action::WaypointManager;
  using GoalHandle = rclcpp_action::ClientGoalHandle<WaypointManagerAction>;

  std::shared_ptr<GoalHandle> goal_handle_;
  std::atomic<bool> cancelled_{false};
};

class StopWaypointManagerState : public yasmin::State {
public:
  StopWaypointManagerState();
  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;
};

}

#endif
