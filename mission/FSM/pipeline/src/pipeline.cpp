#include "pipeline/pipeline.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

FindPipelineState::FindPipelineState(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard)
    : yasmin_ros::ActionState<pipeline_fsm::FindPipelineAction>(
          blackboard->get<std::string>("pose_action"),
          std::bind(&FindPipelineState::create_goal_handler, this, _1),
          std::bind(&FindPipelineState::response_handler, this, _1, _2),
          std::bind(&FindPipelineState::print_feedback, this, _1, _2)) {};

pipeline_fsm::FindPipelineAction::Goal
FindPipelineState::create_goal_handler(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    auto goal = pipeline_fsm::FindPipelineAction::Goal();
    goal.num_measurements = blackboard->get<bool>("num_measurements");

    spdlog::info("Goal sent to action server:");
    spdlog::info("  Number of measurements: {}", goal.num_measurements);

    return goal;
}

std::string FindPipelineState::response_handler(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
    pipeline_fsm::FindPipelineAction::Result::SharedPtr response) {
    blackboard->set<pipeline_fsm::PoseStamped>("pipeline_start_pose",
                                              response->filtered_pose);

    spdlog::info("Response received from action server:");
    spdlog::info("Pipeline start pose: x = {}, y = {}, z = {}",
                 response->filtered_pose.pose.position.x,
                 response->filtered_pose.pose.position.y,
                 response->filtered_pose.pose.position.z);

    pipeline_fsm::PoseStamped pipeline_offset_goal;
    pipeline_offset_goal = response->filtered_pose;
    pipeline_offset_goal.pose.position.z +=
        blackboard->get<double>("pipeline_offset");
    blackboard->set<pipeline_fsm::PoseStamped>("pipeline_offset_goal",
                                              pipeline_offset_goal);

    return yasmin_ros::basic_outcomes::SUCCEED;
}

void FindPipelineState::print_feedback(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
    std::shared_ptr<const pipeline_fsm::FindPipelineAction::Feedback> feedback) {
    blackboard->set<pipeline_fsm::Pose>("current_pose",
                                       feedback->current_pose.pose);
    spdlog::debug("Current position: x = {}, y = {}, z = {}",
                  feedback->current_pose.pose.position.x,
                  feedback->current_pose.pose.position.y,
                  feedback->current_pose.pose.position.z);
}

ApproachPipelineState::ApproachPipelineState(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard)
    : yasmin_ros::ActionState<pipeline_fsm::ApproachPipelineAction>(
          blackboard->get<std::string>("reference_filter_action"),
          std::bind(&ApproachPipelineState::create_goal_handler,
                    this,
                    _1),
          std::bind(&ApproachPipelineState::response_handler,
                    this,
                    _1,
                    _2),
          std::bind(&ApproachPipelineState::print_feedback,
                    this,
                    _1,
                    _2)) {};

pipeline_fsm::ApproachPipelineAction::Goal
ApproachPipelineState::create_goal_handler(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    auto goal = pipeline_fsm::ApproachPipelineAction::Goal();

    blackboard->set<bool>("is_home", false);

    pipeline_fsm::PoseStamped pipeline_offset_goal =
        blackboard->get<pipeline_fsm::PoseStamped>("pipeline_offset_goal");

    goal.goal = pipeline_offset_goal;

    spdlog::info("Goal sent to action server:");
    spdlog::info("  Position: x = {}, y = {}, z = {}",
                 pipeline_offset_goal.pose.position.x,
                 pipeline_offset_goal.pose.position.y,
                 pipeline_offset_goal.pose.position.z);

    return goal;
}

std::string ApproachPipelineState::response_handler(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
    pipeline_fsm::ApproachPipelineAction::Result::SharedPtr response) {
    spdlog::info("Response received from action server:");
    spdlog::info("  Success: {}", response->success ? "true" : "false");

    blackboard->set<bool>("has_finished_converging", response->success);

    if (blackboard->get<bool>("has_finished_converging")) {
        return yasmin_ros::basic_outcomes::SUCCEED;
    } else {
        return yasmin_ros::basic_outcomes::CANCEL;
    }
}

void ApproachPipelineState::print_feedback(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
    std::shared_ptr<const pipeline_fsm::ApproachPipelineAction::Feedback>
        feedback) {
    pipeline_fsm::Pose current_pose = pipeline_fsm::Pose();
    current_pose.position.x = feedback->feedback.x;
    current_pose.position.y = feedback->feedback.y;
    current_pose.position.z = feedback->feedback.z;

    blackboard->set<pipeline_fsm::Pose>("current_pose", current_pose);

    spdlog::debug("Current position: x = {}, y = {}, z = {}",
                  feedback->feedback.x, feedback->feedback.y,
                  feedback->feedback.z);
}

FollowPipelineState::FollowPipelineState(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard)
    : yasmin_ros::ActionState<pipeline_fsm::FollowPipelineAction>(
          blackboard->get<std::string>("reference_filter_action"),
          std::bind(&FollowPipelineState::create_goal_handler, this, _1),
          std::bind(&FollowPipelineState::response_handler,
                    this,
                    _1,
                    _2),
          std::bind(&FollowPipelineState::print_feedback,
                    this,
                    _1,
                    _2)) {};

pipeline_fsm::FollowPipelineAction::Goal
FollowPipelineState::create_goal_handler(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    auto goal = pipeline_fsm::FollowPipelineAction::Goal();

    auto pipeline_offset_goal =
        blackboard->get<pipeline_fsm::PoseStamped>("pipeline_offset_goal");
    goal.goal = pipeline_offset_goal;

    spdlog::info("Goal sent to action server:");
    spdlog::info("  Position: x = {}, y = {}, z = {}",
                 pipeline_offset_goal.pose.position.x,
                 pipeline_offset_goal.pose.position.y,
                 pipeline_offset_goal.pose.position.z);
    spdlog::info("  Orientation: x = {}, y = {}, z = {}, w = {}",
                 pipeline_offset_goal.pose.orientation.x,
                 pipeline_offset_goal.pose.orientation.y,
                 pipeline_offset_goal.pose.orientation.z,
                 pipeline_offset_goal.pose.orientation.w);
    return goal;
}

std::string FollowPipelineState::response_handler(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
    pipeline_fsm::FollowPipelineAction::Result::SharedPtr response) {
    spdlog::info("Response received from action server:");
    spdlog::info("  Success: {}", response->success ? "true" : "false");
    blackboard->set<bool>("is_finished", response->success);

    if (blackboard->get<bool>("is_finished")) {
        return yasmin_ros::basic_outcomes::SUCCEED;
    } else {
        return yasmin_ros::basic_outcomes::CANCEL;
    }
}

void FollowPipelineState::print_feedback(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
    std::shared_ptr<const pipeline_fsm::FollowPipelineAction::Feedback>
        feedback) {
    pipeline_fsm::Pose current_pose = pipeline_fsm::Pose();
    current_pose.position.x = feedback->feedback.x;
    current_pose.position.y = feedback->feedback.y;
    current_pose.position.z = feedback->feedback.z;

    blackboard->set<pipeline_fsm::Pose>("current_pose", current_pose);

    spdlog::debug("Current position: x = {}, y = {}, z = {}",
                  feedback->feedback.x, feedback->feedback.y,
                  feedback->feedback.z);
}


ReturnHomeState::ReturnHomeState(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard)
    : yasmin_ros::ActionState<pipeline_fsm::ReturnHomeAction>(
          blackboard->get<std::string>("reference_filter_action"),
          std::bind(&ReturnHomeState::create_goal_handler, this, _1),
          std::bind(&ReturnHomeState::response_handler, this, _1, _2),
          std::bind(&ReturnHomeState::print_feedback, this, _1, _2)) {};

pipeline_fsm::ReturnHomeAction::Goal ReturnHomeState::create_goal_handler(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    auto goal = pipeline_fsm::ReturnHomeAction::Goal();

    blackboard->set<bool>("is_done", false);

    pipeline_fsm::PoseStamped start_pose =
        blackboard->get<pipeline_fsm::PoseStamped>("start_pose");

    goal.goal = start_pose;
    spdlog::info("Goal sent to action server:");
    spdlog::info("  Position: x = {}, y = {}, z = {}",
                 start_pose.pose.position.x, start_pose.pose.position.y,
                 start_pose.pose.position.z);

    return goal;
}

std::string ReturnHomeState::response_handler(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
    pipeline_fsm::ReturnHomeAction::Result::SharedPtr response) {
    spdlog::info("Response received from action server:");
    spdlog::info("  Success: {}", response->success ? "true" : "false");

    blackboard->set<bool>("is_home", response->success);
    if (blackboard->get<bool>("is_home")) {
        return yasmin_ros::basic_outcomes::SUCCEED;
    } else {
        return yasmin_ros::basic_outcomes::CANCEL;
    }
}

void ReturnHomeState::print_feedback(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
    std::shared_ptr<const pipeline_fsm::ReturnHomeAction::Feedback> feedback) {
    pipeline_fsm::Pose current_pose = pipeline_fsm::Pose();
    current_pose.position.x = feedback->feedback.x;
    current_pose.position.y = feedback->feedback.y;
    current_pose.position.z = feedback->feedback.z;
    current_pose.orientation.x = feedback->feedback.roll;
    current_pose.orientation.y = feedback->feedback.pitch;
    current_pose.orientation.z = feedback->feedback.yaw;

    blackboard->set<pipeline_fsm::Pose>("current_pose", current_pose);
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
    sm->add_state(
        "FIND_PIPELINE",
        std::make_shared<FindPipelineState>(blackboard),
        {
            {yasmin_ros::basic_outcomes::SUCCEED, "APPROACH_PIPELINE"},
            {yasmin_ros::basic_outcomes::ABORT, "ABORT"},
        });
    sm->add_state(
        "APPROACH_PIPELINE",
        std::make_shared<ApproachPipelineState>(blackboard),
        {
            {yasmin_ros::basic_outcomes::SUCCEED, "FOLLOW_PIPELINE"},
            {yasmin_ros::basic_outcomes::CANCEL, "FIND_PIPELINE"},
            {yasmin_ros::basic_outcomes::ABORT, "ABORT"},
        });

    sm->add_state(
        "FOLLOW_PIPELINE",
        std::make_shared<FollowPipelineState>(blackboard),
        {
            {yasmin_ros::basic_outcomes::SUCCEED,
             "RETURN_HOME"},
            {yasmin_ros::basic_outcomes::ABORT, "ABORT"},
            {yasmin_ros::basic_outcomes::CANCEL, "APPROACH_PIPELINE"},

        });


    sm->add_state(
        "RETURN_HOME", std::make_shared<ReturnHomeState>(blackboard),
        {
            {yasmin_ros::basic_outcomes::SUCCEED, "FIND_PIPELINE"},
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
    auto params = std::make_shared<rclcpp::Node>("pipeline_params");
    spdlog::debug("Creating params node");

    params->declare_parameter<double>("fsm.pipeline.pipeline_offset");
    params->declare_parameter<int>("fsm.pipeline.num_measurements");
    

    params->declare_parameter<std::string>("action_servers.reference_filter");
    params->declare_parameter<std::string>("action_servers.los");
    params->declare_parameter<std::string>("action_name");

    spdlog::debug("Parameters declared");

    auto blackboard = std::make_shared<yasmin::blackboard::Blackboard>();

    pipeline_fsm::PoseStamped pipeline_start_pose;
    pipeline_fsm::PoseStamped start_pose;
    blackboard->set<pipeline_fsm::PoseStamped>("pipeline_start_pose", pipeline_start_pose);
    blackboard->set<pipeline_fsm::PoseStamped>("start_pose", start_pose);
    blackboard->set<bool>("return_home", false);
    blackboard->set<bool>("is_home", true);
    blackboard->set<bool>("is_finished", false);
    blackboard->set<bool>("is_error", false);
    blackboard->set<bool>("has_finished_converging", false);
    blackboard->set<int>(
        "num_measurements",
        params->get_parameter("fsm.pipeline.num_measurements").as_int());
    blackboard->set<std::string>(
        "reference_filter_action",
        params->get_parameter("action_servers.reference_filter").as_string());
    blackboard->set<std::string>(
        "los_guidance_action",
        params->get_parameter("action_servers.los").as_string());
    blackboard->set<std::string>(
        "pose_action", params->get_parameter("action_name").as_string());
    blackboard->set<double>(
        "pipeline_offset",
        params->get_parameter("fsm.pipeline.pipeline_offset").as_double());
    blackboard->set<pipeline_fsm::PoseStamped>(
        "pipeline_offset_goal", pipeline_start_pose);
    spdlog::debug("Blackboard created");

    return blackboard;
}

int main(int argc, char* argv[]) {
    spdlog::info("Pipeline");
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

    yasmin_viewer::YasminViewerPub yasmin_pub("Pipeline", sm);

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
