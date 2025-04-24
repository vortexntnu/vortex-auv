#include "structure/structure.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

DetectValveState::DetectValveState(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard)
    : yasmin_ros::ActionState<structure_fsm::DetectValveAction>(
          blackboard->get<std::string>("pose_action"),
          std::bind(&DetectValveState::create_goal_handler, this, _1),
          std::bind(&DetectValveState::response_handler, this, _1, _2),
          std::bind(&DetectValveState::print_feedback, this, _1, _2)) {};

structure_fsm::DetectValveAction::Goal DetectValveState::create_goal_handler(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    auto goal = structure_fsm::DetectValveAction::Goal();
    goal.num_measurements = 5;  // Change

    spdlog::info("Goal sent to action server:");
    spdlog::info("  Number of measurements: 5");

    return goal;
}

std::string DetectValveState::response_handler(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
    structure_fsm::DetectValveAction::Result::SharedPtr response) {
    blackboard->set<structure_fsm::PoseStamped>("dock_pose",
                                                response->filtered_pose);

    spdlog::info("Response received from action server:");
    spdlog::info("Dock pose: x = {}, y = {}, z = {}",
                 response->filtered_pose.pose.position.x,
                 response->filtered_pose.pose.position.y,
                 response->filtered_pose.pose.position.z);

    return yasmin_ros::basic_outcomes::SUCCEED;
}

void DetectValveState::print_feedback(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
    std::shared_ptr<const structure_fsm::DetectValveAction::Feedback>
        feedback) {
    blackboard->set<structure_fsm::Pose>("current_pose",
                                         feedback->current_pose.pose);
    spdlog::debug("Current position: x = {}, y = {}, z = {}",
                  feedback->current_pose.pose.position.x,
                  feedback->current_pose.pose.position.y,
                  feedback->current_pose.pose.position.z);
}

ApproachValveState::ApproachValveState(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard)
    : yasmin_ros::ActionState<structure_fsm::ApproachValveAction>(
          blackboard->get<std::string>("reference_filter_action"),
          std::bind(&ApproachValveState::create_goal_handler, this, _1),
          std::bind(&ApproachValveState::response_handler, this, _1, _2),
          std::bind(&ApproachValveState::print_feedback, this, _1, _2)) {};

structure_fsm::ApproachValveAction::Goal
ApproachValveState::create_goal_handler(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    auto goal = structure_fsm::ApproachValveAction::Goal();

    blackboard->set<bool>("is_home", false);

    goal.goal = structure_fsm::PoseStamped();

    spdlog::info("Goal sent to action server:");
    spdlog::info("  Position: x = {}, y = {}, z = {}",
                 goal.goal.pose.position.x, goal.goal.pose.position.y,
                 goal.goal.pose.position.z);

    return goal;
}

std::string ApproachValveState::response_handler(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
    structure_fsm::ApproachValveAction::Result::SharedPtr response) {
    spdlog::info("Response received from action server:");
    spdlog::info("  Success: {}", response->success ? "true" : "false");

    blackboard->set<bool>("has_finished_converging", response->success);

    if (blackboard->get<bool>("has_finished_converging")) {
        return yasmin_ros::basic_outcomes::SUCCEED;
    } else {
        return yasmin_ros::basic_outcomes::CANCEL;
    }
}

void ApproachValveState::print_feedback(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
    std::shared_ptr<const structure_fsm::ApproachValveAction::Feedback>
        feedback) {
    structure_fsm::Pose current_pose = structure_fsm::Pose();
    current_pose.position.x = feedback->feedback.x;
    current_pose.position.y = feedback->feedback.y;
    current_pose.position.z = feedback->feedback.z;

    blackboard->set<structure_fsm::Pose>("current_pose", current_pose);

    spdlog::debug("Current position: x = {}, y = {}, z = {}",
                  feedback->feedback.x, feedback->feedback.y,
                  feedback->feedback.z);
}

UseGripperState::UseGripperState(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard)
    : yasmin_ros::ActionState<structure_fsm::UseGripperAction>(
          blackboard->get<std::string>("reference_filter_action"),
          std::bind(&UseGripperState::create_goal_handler, this, _1),
          std::bind(&UseGripperState::response_handler, this, _1, _2),
          std::bind(&UseGripperState::print_feedback, this, _1, _2)) {};

structure_fsm::UseGripperAction::Goal UseGripperState::create_goal_handler(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    auto goal = structure_fsm::UseGripperAction::Goal();

    goal.goal = structure_fsm::PoseStamped{};

    spdlog::info("Goal sent to action server:");
    spdlog::info("  Position: x = {}, y = {}, z = {}",
                 goal.goal.pose.position.x, goal.goal.pose.position.y,
                 goal.goal.pose.position.z);
    spdlog::info("  Orientation: x = {}, y = {}, z = {}, w = {}",
                 goal.goal.pose.orientation.x, goal.goal.pose.orientation.y,
                 goal.goal.pose.orientation.z, goal.goal.pose.orientation.w);
    return goal;
}

std::string UseGripperState::response_handler(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
    structure_fsm::UseGripperAction::Result::SharedPtr response) {
    spdlog::info("Response received from action server:");
    spdlog::info("  Success: {}", response->success ? "true" : "false");

    return yasmin_ros::basic_outcomes::SUCCEED;
}

void UseGripperState::print_feedback(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
    std::shared_ptr<const structure_fsm::UseGripperAction::Feedback> feedback) {
    structure_fsm::Pose current_pose = structure_fsm::Pose();
    current_pose.position.x = feedback->feedback.x;
    current_pose.position.y = feedback->feedback.y;
    current_pose.position.z = feedback->feedback.z;

    blackboard->set<structure_fsm::Pose>("current_pose", current_pose);

    spdlog::debug("Current position: x = {}, y = {}, z = {}",
                  feedback->feedback.x, feedback->feedback.y,
                  feedback->feedback.z);
}

GoBackState::GoBackState(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard)
    : yasmin_ros::ActionState<structure_fsm::GoBackAction>(
          blackboard->get<std::string>("reference_filter_action"),
          std::bind(&GoBackState::create_goal_handler, this, _1),
          std::bind(&GoBackState::response_handler, this, _1, _2),
          std::bind(&GoBackState::print_feedback, this, _1, _2)) {};

structure_fsm::GoBackAction::Goal GoBackState::create_goal_handler(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    auto goal = structure_fsm::GoBackAction::Goal();

    goal.goal = structure_fsm::PoseStamped{};

    spdlog::info("Goal sent to action server:");
    spdlog::info("  Position: x = {}, y = {}, z = {}",
                 goal.goal.pose.position.x, goal.goal.pose.position.y,
                 goal.goal.pose.position.z);

    return goal;
}

std::string GoBackState::response_handler(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
    structure_fsm::GoBackAction::Result::SharedPtr response) {
    spdlog::info("Response received from action server:");
    spdlog::info("  Success: {}", response->success ? "true" : "false");

    blackboard->set<bool>("has_finished_converging", response->success);
    if (blackboard->get<bool>("has_finished_converging")) {
        return yasmin_ros::basic_outcomes::SUCCEED;
    } else {
        return yasmin_ros::basic_outcomes::CANCEL;
    }
}

void GoBackState::print_feedback(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
    std::shared_ptr<const structure_fsm::GoBackAction::Feedback> feedback) {
    structure_fsm::Pose current_pose = structure_fsm::Pose();
    current_pose.position.x = feedback->feedback.x;
    current_pose.position.y = feedback->feedback.y;
    current_pose.position.z = feedback->feedback.z;

    blackboard->set<structure_fsm::Pose>("current_pose", current_pose);
    spdlog::debug("Current position: x = {}, y = {}, z = {}",
                  feedback->feedback.x, feedback->feedback.y,
                  feedback->feedback.z);
}

CheckValveTurnedState::CheckValveTurnedState(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard)
    : yasmin_ros::ActionState<structure_fsm::CheckValveTurnedAction>(
          blackboard->get<std::string>("reference_filter_action"),
          std::bind(&CheckValveTurnedState::create_goal_handler, this, _1),
          std::bind(&CheckValveTurnedState::response_handler, this, _1, _2),
          std::bind(&CheckValveTurnedState::print_feedback, this, _1, _2)) {};

structure_fsm::CheckValveTurnedAction::Goal
CheckValveTurnedState::create_goal_handler(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    auto goal = structure_fsm::CheckValveTurnedAction::Goal();

    structure_fsm::PoseStamped start_pose =
        blackboard->get<structure_fsm::PoseStamped>("start_pose");

    goal.goal = start_pose;
    spdlog::info("Goal sent to action server:");
    spdlog::info("  Position: x = {}, y = {}, z = {}",
                 start_pose.pose.position.x, start_pose.pose.position.y,
                 start_pose.pose.position.z);

    return goal;
}

std::string CheckValveTurnedState::response_handler(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
    structure_fsm::CheckValveTurnedAction::Result::SharedPtr response) {
    spdlog::info("Response received from action server:");
    spdlog::info("  Success: {}", response->success ? "true" : "false");

    blackboard->set<bool>("is_home", response->success);
    if (blackboard->get<bool>("is_home")) {
        return yasmin_ros::basic_outcomes::SUCCEED;
    } else {
        return yasmin_ros::basic_outcomes::CANCEL;
    }
}

void CheckValveTurnedState::print_feedback(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
    std::shared_ptr<const structure_fsm::CheckValveTurnedAction::Feedback>
        feedback) {
    structure_fsm::Pose current_pose = structure_fsm::Pose();
    current_pose.position.x = feedback->feedback.x;
    current_pose.position.y = feedback->feedback.y;
    current_pose.position.z = feedback->feedback.z;
    current_pose.orientation.x = feedback->feedback.roll;
    current_pose.orientation.y = feedback->feedback.pitch;
    current_pose.orientation.z = feedback->feedback.yaw;

    blackboard->set<structure_fsm::Pose>("current_pose", current_pose);
    spdlog::debug("Current position: x = {}, y = {}, z = {}",
                  feedback->feedback.x, feedback->feedback.y,
                  feedback->feedback.z);
}

std::string AbortState(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    blackboard->set<bool>("is_abort", true);
    return yasmin_ros::basic_outcomes::ABORT;
};

std::string ErrorState(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    blackboard->set<bool>("is_error", true);
    return yasmin_ros::basic_outcomes::SUCCEED;
};

std::shared_ptr<yasmin::StateMachine> create_state_machines() {
    std::set<std::string> outcomes = {
        "error",
        yasmin_ros::basic_outcomes::SUCCEED,
        yasmin_ros::basic_outcomes::CANCEL,
        yasmin_ros::basic_outcomes::ABORT,
        yasmin_ros::basic_outcomes::TIMEOUT,
    };
    return std::make_shared<yasmin::StateMachine>(outcomes);
}

void add_states(std::shared_ptr<yasmin::StateMachine> sm,
                std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    sm->add_state("DETECT_VALVE",
                  std::make_shared<DetectValveState>(blackboard),
                  {
                      {yasmin_ros::basic_outcomes::SUCCEED, "APPROACH_VALVE"},
                      {yasmin_ros::basic_outcomes::ABORT, "ABORT"},
                  });
    sm->add_state("APPROACH_VALVE",
                  std::make_shared<ApproachValveState>(blackboard),
                  {
                      {yasmin_ros::basic_outcomes::SUCCEED, "USE_GRIPPER"},
                      {yasmin_ros::basic_outcomes::CANCEL, "DETECT_VALVE"},
                      {yasmin_ros::basic_outcomes::ABORT, "ABORT"},
                  });

    sm->add_state("USE_GRIPPER", std::make_shared<UseGripperState>(blackboard),
                  {
                      {yasmin_ros::basic_outcomes::SUCCEED, "GO_BACK"},
                      {yasmin_ros::basic_outcomes::ABORT, "ABORT"},
                      {yasmin_ros::basic_outcomes::CANCEL, "APPROACH_VALVE"},

                  });

    sm->add_state(
        "GO_BACK", std::make_shared<GoBackState>(blackboard),
        {
            {yasmin_ros::basic_outcomes::SUCCEED, "CHECK_VALVE_TURNED"},
            {yasmin_ros::basic_outcomes::ABORT, "ABORT"},
            {yasmin_ros::basic_outcomes::CANCEL, "USE_GRIPPER"},

        });

    sm->add_state("CHECK_VALVE_TURNED",
                  std::make_shared<CheckValveTurnedState>(blackboard),
                  {
                      {yasmin_ros::basic_outcomes::SUCCEED, "DETECT_VALVE"},
                      {yasmin_ros::basic_outcomes::CANCEL, "error"},
                      {yasmin_ros::basic_outcomes::ABORT, "ABORT"},

                  });
    sm->add_state("ABORT",
                  std::make_shared<yasmin::CbState>(
                      std::initializer_list<std::string>{
                          yasmin_ros::basic_outcomes::SUCCEED,
                          yasmin_ros::basic_outcomes::CANCEL,
                          yasmin_ros::basic_outcomes::ABORT},
                      AbortState),
                  {
                      {yasmin_ros::basic_outcomes::SUCCEED,
                       yasmin_ros::basic_outcomes::ABORT},
                      {yasmin_ros::basic_outcomes::CANCEL,
                       yasmin_ros::basic_outcomes::ABORT},
                      {yasmin_ros::basic_outcomes::ABORT,
                       yasmin_ros::basic_outcomes::ABORT},
                  });
    sm->add_state("ERROR",
                  std::make_shared<yasmin::CbState>(
                      std::initializer_list<std::string>{
                          "error", yasmin_ros::basic_outcomes::SUCCEED,
                          yasmin_ros::basic_outcomes::CANCEL,
                          yasmin_ros::basic_outcomes::ABORT},
                      ErrorState),
                  {
                      {yasmin_ros::basic_outcomes::SUCCEED, "error"},
                      {yasmin_ros::basic_outcomes::CANCEL, "error"},
                      {yasmin_ros::basic_outcomes::ABORT, "error"},
                  });
}

auto initialize_blackboard() {
    auto params = std::make_shared<rclcpp::Node>("structure_params");
    spdlog::debug("Creating params node");

    params->declare_parameter<std::string>("action_servers.reference_filter");
    params->declare_parameter<std::string>("action_servers.los");
    params->declare_parameter<std::string>("action_name");

    spdlog::debug("Parameters declared");

    auto blackboard = std::make_shared<yasmin::blackboard::Blackboard>();

    structure_fsm::PoseStamped start_pose;
    blackboard->set<structure_fsm::PoseStamped>("start_pose", start_pose);

    blackboard->set<bool>("is_home", true);
    blackboard->set<bool>("is_error", false);
    blackboard->set<bool>("has_finished_converging", false);
    blackboard->set<std::string>(
        "reference_filter_action",
        params->get_parameter("action_servers.reference_filter").as_string());
    blackboard->set<std::string>(
        "los_guidance_action",
        params->get_parameter("action_servers.los").as_string());
    blackboard->set<std::string>(
        "pose_action", params->get_parameter("action_name").as_string());

    spdlog::debug("Blackboard created");

    return blackboard;
}

int main(int argc, char* argv[]) {
    spdlog::info("Structure");
    rclcpp::init(argc, argv);

    yasmin_ros::set_ros_loggers();

    std::shared_ptr<yasmin::StateMachine> sm = create_state_machines();

    rclcpp::on_shutdown([sm]() {
        if (sm->is_running()) {
            sm->cancel_state();
        }
    });

    auto blackboard = initialize_blackboard();

    add_states(sm, blackboard);

    yasmin_viewer::YasminViewerPub yasmin_pub("Structure", sm);

    spdlog::debug("State machines created");

    try {
        std::string outcome = (*sm.get())(blackboard);
        spdlog::info("State machine finished with outcome: {}", outcome);
    } catch (const std::exception& e) {
        spdlog::warn(e.what());
        rcutils_reset_error();
    }

    if (!rclcpp::ok()) {
        spdlog::info(
            "ROS2 context is already invalid. Skipping publisher destruction.");
        return 1;
    }

    if (rclcpp::ok()) {
        sm.reset();
        blackboard.reset();

        rclcpp::shutdown();
        spdlog::info("ROS2 shutdown completed gracefully.");
    }

    return 0;
}
