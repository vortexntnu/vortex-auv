#include "docking/docking.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

FindDockingStationState::FindDockingStationState(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard)
    : yasmin_ros::ActionState<docking_fsm::FindDockingAction>(
          blackboard->get<std::string>("pose_action"),
          std::bind(&FindDockingStationState::create_goal_handler, this, _1),
          std::bind(&FindDockingStationState::response_handler, this, _1, _2),
          std::bind(&FindDockingStationState::print_feedback, this, _1, _2)) {}

docking_fsm::FindDockingAction::Goal
FindDockingStationState::create_goal_handler(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    auto goal = docking_fsm::FindDockingAction::Goal();
    goal.num_measurements = blackboard->get<bool>("num_measurements");

    spdlog::info("Goal sent to action server:");
    spdlog::info("  Number of measurements: {}", goal.num_measurements);

    return goal;
}

std::string FindDockingStationState::response_handler(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
    docking_fsm::FindDockingAction::Result::SharedPtr response) {
    blackboard->set<docking_fsm::PoseStamped>("dock_pose",
                                              response->filtered_pose);

    spdlog::info("Response received from action server:");
    spdlog::info("Dock pose: x = {}, y = {}, z = {}",
                 response->filtered_pose.pose.position.x,
                 response->filtered_pose.pose.position.y,
                 response->filtered_pose.pose.position.z);

    docking_fsm::PoseStamped docking_offset_goal;
    docking_offset_goal = response->filtered_pose;
    docking_offset_goal.pose.position.z +=
        blackboard->get<double>("docking_station_offset");
    blackboard->set<docking_fsm::PoseStamped>("docking_offset_goal",
                                              docking_offset_goal);

    return yasmin_ros::basic_outcomes::SUCCEED;
}

void FindDockingStationState::print_feedback(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
    std::shared_ptr<const docking_fsm::FindDockingAction::Feedback> feedback) {
    blackboard->set<docking_fsm::Pose>("current_pose",
                                       feedback->current_pose.pose);
    spdlog::debug("Current position: x = {}, y = {}, z = {}",
                  feedback->current_pose.pose.position.x,
                  feedback->current_pose.pose.position.y,
                  feedback->current_pose.pose.position.z);
}

ApproachDockingStationState::ApproachDockingStationState(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard)
    : yasmin_ros::ActionState<docking_fsm::ApproachDockingAction>(
          blackboard->get<std::string>("reference_filter_action"),
          std::bind(&ApproachDockingStationState::create_goal_handler,
                    this,
                    _1),
          std::bind(&ApproachDockingStationState::response_handler,
                    this,
                    _1,
                    _2),
          std::bind(&ApproachDockingStationState::print_feedback,
                    this,
                    _1,
                    _2)) {}

docking_fsm::ApproachDockingAction::Goal
ApproachDockingStationState::create_goal_handler(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    auto goal = docking_fsm::ApproachDockingAction::Goal();

    blackboard->set<bool>("is_home", false);

    docking_fsm::Pose docking_offset_goal =
        blackboard->get<docking_fsm::PoseStamped>("docking_offset_goal").pose;

    vortex_msgs::msg::Waypoint waypoint;
    waypoint.pose = docking_offset_goal;

    goal.waypoint = waypoint;

    spdlog::info("Goal sent to action server:");
    spdlog::info("  Position: x = {}, y = {}, z = {}",
                 docking_offset_goal.position.x, docking_offset_goal.position.y,
                 docking_offset_goal.position.z);

    return goal;
}

std::string ApproachDockingStationState::response_handler(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
    docking_fsm::ApproachDockingAction::Result::SharedPtr response) {
    spdlog::info("Response received from action server:");
    spdlog::info("  Success: {}", response->success ? "true" : "false");

    blackboard->set<bool>("has_finished_converging", response->success);

    if (blackboard->get<bool>("has_finished_converging")) {
        return yasmin_ros::basic_outcomes::SUCCEED;
    } else {
        return yasmin_ros::basic_outcomes::CANCEL;
    }
}

void ApproachDockingStationState::print_feedback(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
    std::shared_ptr<const docking_fsm::ApproachDockingAction::Feedback>
        feedback) {
    docking_fsm::Pose current_pose = docking_fsm::Pose();
    current_pose.position.x = feedback->reference.x;
    current_pose.position.y = feedback->reference.y;
    current_pose.position.z = feedback->reference.z;

    blackboard->set<docking_fsm::Pose>("current_pose", current_pose);

    spdlog::debug("Current position: x = {}, y = {}, z = {}",
                  feedback->reference.x, feedback->reference.y,
                  feedback->reference.z);
}

GoAboveDockingStationState::GoAboveDockingStationState(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard)
    : yasmin_ros::ActionState<docking_fsm::GoAboveDockingAction>(
          blackboard->get<std::string>("reference_filter_action"),
          std::bind(&GoAboveDockingStationState::create_goal_handler, this, _1),
          std::bind(&GoAboveDockingStationState::response_handler,
                    this,
                    _1,
                    _2),
          std::bind(&GoAboveDockingStationState::print_feedback,
                    this,
                    _1,
                    _2)) {}

docking_fsm::GoAboveDockingAction::Goal
GoAboveDockingStationState::create_goal_handler(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    auto goal = docking_fsm::GoAboveDockingAction::Goal();

    auto docking_offset_goal =
        blackboard->get<docking_fsm::PoseStamped>("docking_offset_goal");
    vortex_msgs::msg::Waypoint waypoint;
    waypoint.pose = docking_offset_goal.pose;
    goal.waypoint = waypoint;

    spdlog::info("Goal sent to action server:");
    spdlog::info("  Position: x = {}, y = {}, z = {}",
                 docking_offset_goal.pose.position.x,
                 docking_offset_goal.pose.position.y,
                 docking_offset_goal.pose.position.z);
    spdlog::info("  Orientation: x = {}, y = {}, z = {}, w = {}",
                 docking_offset_goal.pose.orientation.x,
                 docking_offset_goal.pose.orientation.y,
                 docking_offset_goal.pose.orientation.z,
                 docking_offset_goal.pose.orientation.w);
    return goal;
}

std::string GoAboveDockingStationState::response_handler(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
    docking_fsm::GoAboveDockingAction::Result::SharedPtr response) {
    spdlog::info("Response received from action server:");
    spdlog::info("  Success: {}", response->success ? "true" : "false");
    blackboard->set<bool>("is_docked", response->success);

    if (blackboard->get<bool>("is_docked")) {
        return yasmin_ros::basic_outcomes::SUCCEED;
    } else {
        return yasmin_ros::basic_outcomes::CANCEL;
    }
}

void GoAboveDockingStationState::print_feedback(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
    std::shared_ptr<const docking_fsm::GoAboveDockingAction::Feedback>
        feedback) {
    docking_fsm::Pose current_pose = docking_fsm::Pose();
    current_pose.position.x = feedback->reference.x;
    current_pose.position.y = feedback->reference.y;
    current_pose.position.z = feedback->reference.z;

    blackboard->set<docking_fsm::Pose>("current_pose", current_pose);

    spdlog::debug("Current position: x = {}, y = {}, z = {}",
                  feedback->reference.x, feedback->reference.y,
                  feedback->reference.z);
}

ConvergeDockingStationState::ConvergeDockingStationState(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard)
    : yasmin_ros::ActionState<docking_fsm::ConvergeDockingAction>(
          blackboard->get<std::string>("reference_filter_action"),
          std::bind(&ConvergeDockingStationState::create_goal_handler,
                    this,
                    _1),
          std::bind(&ConvergeDockingStationState::response_handler,
                    this,
                    _1,
                    _2),
          std::bind(&ConvergeDockingStationState::print_feedback,
                    this,
                    _1,
                    _2)) {}

docking_fsm::ConvergeDockingAction::Goal
ConvergeDockingStationState::create_goal_handler(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    auto goal = docking_fsm::ConvergeDockingAction::Goal();

    docking_fsm::PoseStamped dock_pose =
        blackboard->get<docking_fsm::PoseStamped>("dock_pose");
    vortex_msgs::msg::Waypoint waypoint;
    waypoint.pose = dock_pose.pose;
    goal.waypoint = waypoint;

    spdlog::info("Goal sent to action server:");
    spdlog::info("  Position: x = {}, y = {}, z = {}",
                 dock_pose.pose.position.x, dock_pose.pose.position.y,
                 dock_pose.pose.position.z);

    return goal;
}

std::string ConvergeDockingStationState::response_handler(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
    docking_fsm::ConvergeDockingAction::Result::SharedPtr response) {
    spdlog::info("Response received from action server:");
    spdlog::info("  Success: {}", response->success ? "true" : "false");

    blackboard->set<bool>("has_finished_converging", response->success);
    if (blackboard->get<bool>("has_finished_converging")) {
        return yasmin_ros::basic_outcomes::SUCCEED;
    } else {
        return yasmin_ros::basic_outcomes::CANCEL;
    }
}

void ConvergeDockingStationState::print_feedback(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
    std::shared_ptr<const docking_fsm::ConvergeDockingAction::Feedback>
        feedback) {
    docking_fsm::Pose current_pose = docking_fsm::Pose();
    current_pose.position.x = feedback->reference.x;
    current_pose.position.y = feedback->reference.y;
    current_pose.position.z = feedback->reference.z;

    blackboard->set<docking_fsm::Pose>("current_pose", current_pose);
    spdlog::debug("Current position: x = {}, y = {}, z = {}",
                  feedback->reference.x, feedback->reference.y,
                  feedback->reference.z);
}

std::string DockedState(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    std::chrono::duration<double> elapsed_time = std::chrono::seconds(0);
    std::chrono::time_point<std::chrono::steady_clock> start_time =
        std::chrono::steady_clock::now();

    while (true) {
        elapsed_time = (std::chrono::steady_clock::now() - start_time);

        if (elapsed_time.count() > blackboard->get<double>("dock_wait_time")) {
            return yasmin_ros::basic_outcomes::SUCCEED;
        } else if (blackboard->get<bool>("is_error")) {
            return yasmin_ros::basic_outcomes::ABORT;
        }
    }
}

ReturnHomeState::ReturnHomeState(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard)
    : yasmin_ros::ActionState<docking_fsm::ReturnHomeAction>(
          blackboard->get<std::string>("reference_filter_action"),
          std::bind(&ReturnHomeState::create_goal_handler, this, _1),
          std::bind(&ReturnHomeState::response_handler, this, _1, _2),
          std::bind(&ReturnHomeState::print_feedback, this, _1, _2)) {}

docking_fsm::ReturnHomeAction::Goal ReturnHomeState::create_goal_handler(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    auto goal = docking_fsm::ReturnHomeAction::Goal();

    blackboard->set<bool>("is_docked", false);

    docking_fsm::PoseStamped start_pose =
        blackboard->get<docking_fsm::PoseStamped>("start_pose");

    vortex_msgs::msg::Waypoint waypoint;
    waypoint.pose = start_pose.pose;
    goal.waypoint = waypoint;
    spdlog::info("Goal sent to action server:");
    spdlog::info("  Position: x = {}, y = {}, z = {}",
                 start_pose.pose.position.x, start_pose.pose.position.y,
                 start_pose.pose.position.z);

    return goal;
}

std::string ReturnHomeState::response_handler(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
    docking_fsm::ReturnHomeAction::Result::SharedPtr response) {
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
    std::shared_ptr<const docking_fsm::ReturnHomeAction::Feedback> feedback) {
    docking_fsm::Pose current_pose = docking_fsm::Pose();
    current_pose.position.x = feedback->reference.x;
    current_pose.position.y = feedback->reference.y;
    current_pose.position.z = feedback->reference.z;
    current_pose.orientation.x = feedback->reference.roll;
    current_pose.orientation.y = feedback->reference.pitch;
    current_pose.orientation.z = feedback->reference.yaw;

    blackboard->set<docking_fsm::Pose>("current_pose", current_pose);
    spdlog::debug("Current position: x = {}, y = {}, z = {}",
                  feedback->reference.x, feedback->reference.y,
                  feedback->reference.z);
}

std::string AbortState(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    blackboard->set<bool>("is_abort", true);
    return yasmin_ros::basic_outcomes::ABORT;
}

std::string ErrorState(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    blackboard->set<bool>("is_error", true);
    return yasmin_ros::basic_outcomes::SUCCEED;
}

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
        "FIND_DOCKING_STATION",
        std::make_shared<FindDockingStationState>(blackboard),
        {
            {yasmin_ros::basic_outcomes::SUCCEED, "APPROACH_DOCKING_STATION"},
            {yasmin_ros::basic_outcomes::ABORT, "ABORT"},
        });
    sm->add_state(
        "APPROACH_DOCKING_STATION",
        std::make_shared<ApproachDockingStationState>(blackboard),
        {
            {yasmin_ros::basic_outcomes::SUCCEED, "GO_ABOVE_DOCKING_STATION"},
            {yasmin_ros::basic_outcomes::CANCEL, "FIND_DOCKING_STATION"},
            {yasmin_ros::basic_outcomes::ABORT, "ABORT"},
        });

    sm->add_state(
        "GO_ABOVE_DOCKING_STATION",
        std::make_shared<GoAboveDockingStationState>(blackboard),
        {
            {yasmin_ros::basic_outcomes::SUCCEED,
             "UPDATE_DOCKING_STATION_POSITION"},
            {yasmin_ros::basic_outcomes::ABORT, "ABORT"},
            {yasmin_ros::basic_outcomes::CANCEL, "APPROACH_DOCKING_STATION"},
        });

    sm->add_state(
        "UPDATE_DOCKING_STATION_POSITION",
        std::make_shared<FindDockingStationState>(blackboard),
        {
            {yasmin_ros::basic_outcomes::SUCCEED, "CONVERGE_DOCKING_STATION"},
            {yasmin_ros::basic_outcomes::ABORT, "ABORT"},
            {yasmin_ros::basic_outcomes::CANCEL, "GO_ABOVE_DOCKING_STATION"},
        });

    sm->add_state(
        "CONVERGE_DOCKING_STATION",
        std::make_shared<ConvergeDockingStationState>(blackboard),
        {
            {yasmin_ros::basic_outcomes::SUCCEED, "DOCKED"},
            {yasmin_ros::basic_outcomes::ABORT, "ABORT"},
            {yasmin_ros::basic_outcomes::CANCEL, "GO_ABOVE_DOCKING_STATION"},
        });

    sm->add_state("DOCKED",
                  std::make_shared<yasmin::CbState>(
                      std::initializer_list<std::string>{
                          "error", yasmin_ros::basic_outcomes::SUCCEED,
                          yasmin_ros::basic_outcomes::ABORT},
                      DockedState),
                  {
                      {yasmin_ros::basic_outcomes::SUCCEED,
                       yasmin_ros::basic_outcomes::SUCCEED},
                      {yasmin_ros::basic_outcomes::ABORT, "ABORT"},
                  });
    sm->add_state(
        "RETURN_HOME", std::make_shared<ReturnHomeState>(blackboard),
        {
            {yasmin_ros::basic_outcomes::SUCCEED, "FIND_DOCKING_STATION"},
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
    auto params = std::make_shared<rclcpp::Node>("dock_params");
    spdlog::debug("Creating params node");

    params->declare_parameter<double>("fsm.docking.docking_station_offset");
    params->declare_parameter<int>("fsm.docking.num_measurements");
    params->declare_parameter<double>("fsm.docking.dock_wait_time");

    params->declare_parameter<std::string>("action_servers.reference_filter");
    params->declare_parameter<std::string>("action_servers.los");
    params->declare_parameter<std::string>("action_name");

    spdlog::debug("Parameters declared");

    auto blackboard = std::make_shared<yasmin::blackboard::Blackboard>();

    docking_fsm::PoseStamped dock_pose;
    docking_fsm::PoseStamped start_pose;
    blackboard->set<docking_fsm::PoseStamped>("dock_pose", dock_pose);
    blackboard->set<docking_fsm::PoseStamped>("start_pose", start_pose);

    blackboard->set<double>(
        "docking_station_offset",
        params->get_parameter("fsm.docking.docking_station_offset")
            .as_double());
    blackboard->set<bool>("return_home", false);
    blackboard->set<bool>("is_docked", false);
    blackboard->set<bool>("is_home", true);
    blackboard->set<bool>("is_error", false);
    blackboard->set<bool>("has_finished_converging", false);
    blackboard->set<int>(
        "num_measurements",
        params->get_parameter("fsm.docking.num_measurements").as_int());
    blackboard->set<std::string>(
        "reference_filter_action",
        params->get_parameter("action_servers.reference_filter").as_string());
    blackboard->set<double>(
        "dock_wait_time",
        params->get_parameter("fsm.docking.dock_wait_time").as_double());
    blackboard->set<std::string>(
        "los_guidance_action",
        params->get_parameter("action_servers.los").as_string());
    blackboard->set<std::string>(
        "pose_action", params->get_parameter("action_name").as_string());

    spdlog::debug("Blackboard created");

    return blackboard;
}

int main(int argc, char* argv[]) {
    spdlog::info("Docking");
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

    yasmin_viewer::YasminViewerPub yasmin_pub("Docking", sm);

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
