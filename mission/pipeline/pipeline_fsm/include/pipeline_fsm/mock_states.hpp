#ifndef PIPELINE_FSM__MOCK_STATES_HPP_
#define PIPELINE_FSM__MOCK_STATES_HPP_

#include "yasmin/blackboard.hpp"
#include "yasmin/state.hpp"
#include "yasmin_ros/basic_outcomes.hpp"

namespace pipeline_fsm {

class MockSearchPatternState : public yasmin::State {
public:
  MockSearchPatternState();
  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;

private:
  int counter_;
};

class MockPipelineFollowState : public yasmin::State {
public:
  MockPipelineFollowState();
  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;

private:
  int counter_;
};

class MockConvergeState : public yasmin::State {
public:
  MockConvergeState();
  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;

private:
  int counter_;
};

class IdleState : public yasmin::State {
public:
  IdleState();
  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;
};

class DoneState : public yasmin::State {
public:
  DoneState();
  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;
};

class AbortedState : public yasmin::State {
public:
  AbortedState();
  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;
};

}

#endif
