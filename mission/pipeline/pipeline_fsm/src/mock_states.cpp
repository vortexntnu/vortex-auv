#include "pipeline_fsm/mock_states.hpp"
#include "yasmin_ros/ros_logs.hpp"
#include <sstream>

namespace pipeline_fsm {

MockSearchPatternState::MockSearchPatternState()
    : yasmin::State({yasmin_ros::basic_outcomes::SUCCEED,
                     yasmin_ros::basic_outcomes::ABORT}),
      counter_(0) {}

std::string
MockSearchPatternState::execute(yasmin::Blackboard::SharedPtr blackboard) {
  YASMIN_LOG_INFO("Executing SEARCH_PATTERN");

  counter_ = 0;
  while (counter_ < 150 && !this->is_canceled()) {
    counter_++;
    if (counter_ % 30 == 0) {
      std::stringstream ss;
      ss << "SEARCH_PATTERN progress: " << counter_ << "/150 (" 
         << (counter_ * 100 / 150) << "%)";
      YASMIN_LOG_INFO(ss.str().c_str());
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  if (this->is_canceled()) {
    YASMIN_LOG_INFO("SEARCH_PATTERN cancelled");
    return yasmin_ros::basic_outcomes::ABORT;
  }

  YASMIN_LOG_INFO("SEARCH_PATTERN completed");
  return yasmin_ros::basic_outcomes::SUCCEED;
}

MockPipelineFollowState::MockPipelineFollowState()
    : yasmin::State({yasmin_ros::basic_outcomes::SUCCEED,
                     yasmin_ros::basic_outcomes::ABORT}),
      counter_(0) {}

std::string
MockPipelineFollowState::execute(yasmin::Blackboard::SharedPtr blackboard) {
  YASMIN_LOG_INFO("Executing PIPELINE_FOLLOW_ACTION");

  counter_ = 0;
  while (counter_ < 150 && !this->is_canceled()) {
    counter_++;
    if (counter_ % 30 == 0) {
      std::stringstream ss;
      ss << "PIPELINE_FOLLOW_ACTION progress: " << counter_ << "/150 (" 
         << (counter_ * 100 / 150) << "%)";
      YASMIN_LOG_INFO(ss.str().c_str());
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  if (this->is_canceled()) {
    YASMIN_LOG_INFO("PIPELINE_FOLLOW_ACTION cancelled");
    return yasmin_ros::basic_outcomes::ABORT;
  }

  YASMIN_LOG_INFO("PIPELINE_FOLLOW_ACTION completed");
  return yasmin_ros::basic_outcomes::SUCCEED;
}

MockConvergeState::MockConvergeState()
    : yasmin::State({yasmin_ros::basic_outcomes::SUCCEED,
                     yasmin_ros::basic_outcomes::ABORT}),
      counter_(0) {}

std::string
MockConvergeState::execute(yasmin::Blackboard::SharedPtr blackboard) {
  YASMIN_LOG_INFO("Executing CONVERGE");

  counter_ = 0;
  while (counter_ < 150 && !this->is_canceled()) {
    counter_++;
    if (counter_ % 30 == 0) {
      std::stringstream ss;
      ss << "CONVERGE progress: " << counter_ << "/150 (" 
         << (counter_ * 100 / 150) << "%)";
      YASMIN_LOG_INFO(ss.str().c_str());
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  if (this->is_canceled()) {
    YASMIN_LOG_INFO("CONVERGE cancelled");
    return yasmin_ros::basic_outcomes::ABORT;
  }

  YASMIN_LOG_INFO("CONVERGE completed");
  return yasmin_ros::basic_outcomes::SUCCEED;
}

IdleState::IdleState()
    : yasmin::State({yasmin_ros::basic_outcomes::SUCCEED}) {}

std::string IdleState::execute(yasmin::Blackboard::SharedPtr blackboard) {
  YASMIN_LOG_INFO("Executing IDLE");
  return yasmin_ros::basic_outcomes::SUCCEED;
}

DoneState::DoneState() : yasmin::State({"done"}) {}

std::string DoneState::execute(yasmin::Blackboard::SharedPtr blackboard) {
  YASMIN_LOG_INFO("Mission DONE");
  return "done";
}

AbortedState::AbortedState() : yasmin::State({"aborted"}) {}

std::string AbortedState::execute(yasmin::Blackboard::SharedPtr blackboard) {
  YASMIN_LOG_INFO("Mission ABORTED");
  return "aborted";
}

}
