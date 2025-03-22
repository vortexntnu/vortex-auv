#include "docking_cpp/docking.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

FindDockingStationState::FindDockingStationState()
    : yasmin_ros::ActionState<docking_fsm::FilteredPose>(
          "filtered_pose",
          std::bind(&FindDockingStationState::create_goal_handler, this, _1),
          std::bind(&FindDockingStationState::response_handler, this, _1, _2),
          std::bind(&FindDockingStationState::print_feedback, this, _1, _2)) {};

docking_fsm::FilteredPose::Goal FindDockingStationState::create_goal_handler(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    auto goal = docking_fsm::FilteredPose::Goal();
    goal.num_measurements = blackboard->get<bool>("num_measurements");

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal sent to action server:");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  Number of measurements: %d",
                goal.num_measurements);

    return goal;
}

std::string FindDockingStationState::response_handler(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
    docking_fsm::FilteredPose::Result::SharedPtr response) {
    blackboard->set<docking_fsm::PoseStamped>("dock_pose",
                                              response->filtered_pose);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Response received from action server:");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "  Dock pose: x = %f, y = %f, z = %f\n",
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
    std::shared_ptr<const docking_fsm::FilteredPose::Feedback> feedback) {
    blackboard->set<docking_fsm::Pose>("current_pose",
                                       feedback->current_pose.pose);

    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
                 "Current position: x = %f, y = %f, z = %f\n",
                 feedback->current_pose.pose.position.x,
                 feedback->current_pose.pose.position.y,
                 feedback->current_pose.pose.position.z);
}

ApproachDockingStationState::ApproachDockingStationState()
    : yasmin_ros::ActionState<docking_fsm::LOSGuidance>(
          "/los_guidance",
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
                    _2)) {};

docking_fsm::LOSGuidance::Goal ApproachDockingStationState::create_goal_handler(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    auto goal = docking_fsm::LOSGuidance::Goal();

    blackboard->set<bool>("is_home", false);

    docking_fsm::PoseStamped docking_offset_goal =
        blackboard->get<docking_fsm::PoseStamped>("docking_offset_goal");

    docking_fsm::PointStamped docking_point;
    docking_point.point = docking_offset_goal.pose.position;
    docking_point.header = docking_offset_goal.header;

    goal.goal = docking_point;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal sent to action server:");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "  Position: x = %f, y = %f, z = %f", docking_point.point.x,
                docking_point.point.y, docking_point.point.z);

    return goal;
}

std::string ApproachDockingStationState::response_handler(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
    docking_fsm::LOSGuidance::Result::SharedPtr response) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Response received from action server:");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  Success: %s\n",
                response->success ? "true" : "false");

    blackboard->set<bool>("has_finished_converging", response->success);

    if (blackboard->get<bool>("has_finished_converging")) {
        return yasmin_ros::basic_outcomes::SUCCEED;
    } else {
        return yasmin_ros::basic_outcomes::CANCEL;
    }
}

void ApproachDockingStationState::print_feedback(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
    std::shared_ptr<const docking_fsm::LOSGuidance::Feedback> feedback) {
    blackboard->set<vortex_msgs::msg::LOSGuidance>("current_LOS",
                                                   feedback->feedback);

    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
                 "Current surge, pitch and yaw: %f, %f, %f",
                 feedback->feedback.surge, feedback->feedback.pitch,
                 feedback->feedback.yaw);
}

GoAboveDockingStationState::GoAboveDockingStationState(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard)
    : yasmin_ros::ActionState<docking_fsm::ReferenceFilterWaypoint>(
          blackboard->get<std::string>("reference_filter_action"),
          std::bind(&GoAboveDockingStationState::create_goal_handler, this, _1),
          std::bind(&GoAboveDockingStationState::response_handler,
                    this,
                    _1,
                    _2),
          std::bind(&GoAboveDockingStationState::print_feedback,
                    this,
                    _1,
                    _2)) {};

docking_fsm::ReferenceFilterWaypoint::Goal
GoAboveDockingStationState::create_goal_handler(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    auto goal = docking_fsm::ReferenceFilterWaypoint::Goal();

    auto docking_offset_goal =
        blackboard->get<docking_fsm::PoseStamped>("docking_offset_goal");
    goal.goal = docking_offset_goal;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal sent to action server:");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "  Position: x = %f, y = %f, z = %f",
                docking_offset_goal.pose.position.x,
                docking_offset_goal.pose.position.y,
                docking_offset_goal.pose.position.z);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "  Orientation: x = %f, y = %f, z = %f, w = %f\n",
                docking_offset_goal.pose.orientation.x,
                docking_offset_goal.pose.orientation.y,
                docking_offset_goal.pose.orientation.z,
                docking_offset_goal.pose.orientation.w);

    return goal;
}

std::string GoAboveDockingStationState::response_handler(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
    docking_fsm::ReferenceFilterWaypoint::Result::SharedPtr response) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Response received from action server:");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  Success: %s\n",
                response->success ? "true" : "false");

    blackboard->set<bool>("is_docked", response->success);

    if (blackboard->get<bool>("is_docked")) {
        return yasmin_ros::basic_outcomes::SUCCEED;
    } else {
        return yasmin_ros::basic_outcomes::CANCEL;
    }
}

void GoAboveDockingStationState::print_feedback(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
    std::shared_ptr<const docking_fsm::ReferenceFilterWaypoint::Feedback>
        feedback) {
    docking_fsm::Pose current_pose = docking_fsm::Pose();
    current_pose.position.x = feedback->feedback.x;
    current_pose.position.y = feedback->feedback.y;
    current_pose.position.z = feedback->feedback.z;

    blackboard->set<docking_fsm::Pose>("current_pose", current_pose);

    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
                 "Current position: x = %f, y = %f, z = %f\n",
                 feedback->feedback.x, feedback->feedback.y,
                 feedback->feedback.z);
}

ConvergeDockingStationState::ConvergeDockingStationState()
    : yasmin_ros::ActionState<docking_fsm::ReferenceFilterWaypoint>(
          "/reference_filter",
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
                    _2)) {};

docking_fsm::ReferenceFilterWaypoint::Goal
ConvergeDockingStationState::create_goal_handler(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    auto goal = docking_fsm::ReferenceFilterWaypoint::Goal();

    docking_fsm::PoseStamped dock_pose =
        blackboard->get<docking_fsm::PoseStamped>("dock_pose");
    goal.goal = dock_pose;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal sent to action server:");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "  Position: x = %f, y = %f, z = %f\n",
                goal.goal.pose.position.x, goal.goal.pose.position.y,
                goal.goal.pose.position.z);

    return goal;
}

std::string ConvergeDockingStationState::response_handler(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
    docking_fsm::ReferenceFilterWaypoint::Result::SharedPtr response) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Response received from action server:");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  Success: %s\n",
                response->success ? "true" : "false");

    blackboard->set<bool>("has_finished_converging", response->success);
    if (blackboard->get<bool>("has_finished_converging")) {
        return yasmin_ros::basic_outcomes::SUCCEED;
    } else {
        return yasmin_ros::basic_outcomes::CANCEL;
    }
}

void ConvergeDockingStationState::print_feedback(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
    std::shared_ptr<const docking_fsm::ReferenceFilterWaypoint::Feedback>
        feedback) {
    docking_fsm::Pose current_pose = docking_fsm::Pose();
    current_pose.position.x = feedback->feedback.x;
    current_pose.position.y = feedback->feedback.y;
    current_pose.position.z = feedback->feedback.z;

    blackboard->set<docking_fsm::Pose>("current_pose", current_pose);

    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
                 "Current position: x = %f, y = %f, z = %f\n",
                 feedback->feedback.x, feedback->feedback.y,
                 feedback->feedback.z);
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
};

ReturnHomeState::ReturnHomeState()
    : yasmin_ros::ActionState<docking_fsm::ReferenceFilterWaypoint>(
          "/reference_filter",
          std::bind(&ReturnHomeState::create_goal_handler, this, _1),
          std::bind(&ReturnHomeState::response_handler, this, _1, _2),
          std::bind(&ReturnHomeState::print_feedback, this, _1, _2)) {};

docking_fsm::ReferenceFilterWaypoint::Goal ReturnHomeState::create_goal_handler(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    auto goal = docking_fsm::ReferenceFilterWaypoint::Goal();

    blackboard->set<bool>("is_docked", false);

    docking_fsm::PoseStamped start_pose =
        blackboard->get<docking_fsm::PoseStamped>("start_pose");

    goal.goal = start_pose;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal sent to action server:");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "  Start position: x = %f, y = %f, z = %f\n",
                start_pose.pose.position.x, start_pose.pose.position.y,
                start_pose.pose.position.z);

    return goal;
}

std::string ReturnHomeState::response_handler(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
    docking_fsm::ReferenceFilterWaypoint::Result::SharedPtr response) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Response received from action server:");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  Success: %s\n",
                response->success ? "true" : "false");

    blackboard->set<bool>("is_home", response->success);
    if (blackboard->get<bool>("is_home")) {
        return yasmin_ros::basic_outcomes::SUCCEED;
    } else {
        return yasmin_ros::basic_outcomes::CANCEL;
    }
}

void ReturnHomeState::print_feedback(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
    std::shared_ptr<const docking_fsm::ReferenceFilterWaypoint::Feedback>
        feedback) {
    docking_fsm::Pose current_pose = docking_fsm::Pose();
    current_pose.position.x = feedback->feedback.x;
    current_pose.position.y = feedback->feedback.y;
    current_pose.position.z = feedback->feedback.z;
    current_pose.orientation.x = feedback->feedback.roll;
    current_pose.orientation.y = feedback->feedback.pitch;
    current_pose.orientation.z = feedback->feedback.yaw;

    blackboard->set<docking_fsm::Pose>("current_pose", current_pose);
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
                 "Current position: x = %f, y = %f, z = %f\n",
                 current_pose.position.x, current_pose.position.y,
                 current_pose.position.z);
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
                std::shared_ptr<yasmin::StateMachine> nested_sm) {
    sm->add_state(
        "FIND_DOCKING_STATION", std::make_shared<FindDockingStationState>(),
        {
            {yasmin_ros::basic_outcomes::SUCCEED, "APPROACH_DOCKING_STATION"},
            {yasmin_ros::basic_outcomes::CANCEL, "error"},
            {yasmin_ros::basic_outcomes::ABORT, "ABORT"},
        });
    sm->add_state(
        "APPROACH_DOCKING_STATION",
        std::make_shared<ApproachDockingStationState>(),
        {
            {yasmin_ros::basic_outcomes::SUCCEED, "NESTED_FSM"},
            {yasmin_ros::basic_outcomes::CANCEL, "FIND_DOCKING_STATION"},
            {yasmin_ros::basic_outcomes::ABORT, "ABORT"},
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
        "RETURN_HOME", std::make_shared<ReturnHomeState>(),
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
    sm->add_state(
        "NESTED_FSM", nested_sm,
        {
            {yasmin_ros::basic_outcomes::SUCCEED, "DOCKED"},
            {yasmin_ros::basic_outcomes::CANCEL, "APPROACH_DOCKING_STATION"},
            {yasmin_ros::basic_outcomes::ABORT, "ABORT"},

        });
}

void add_states_nested(
    std::shared_ptr<yasmin::StateMachine> sm,
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    sm->add_state("GO_ABOVE_DOCKING_STATION",
                  std::make_shared<GoAboveDockingStationState>(blackboard),
                  {
                      {yasmin_ros::basic_outcomes::SUCCEED,
                       "UPDATE_DOCKING_STATION_POSITION"},
                      {yasmin_ros::basic_outcomes::ABORT,
                       yasmin_ros::basic_outcomes::ABORT},
                      {yasmin_ros::basic_outcomes::CANCEL,
                       yasmin_ros::basic_outcomes::CANCEL},

                  });

    sm->add_state(
        "UPDATE_DOCKING_STATION_POSITION",
        std::make_shared<FindDockingStationState>(),
        {
            {yasmin_ros::basic_outcomes::SUCCEED, "CONVERGE_DOCKING_STATION"},
            {yasmin_ros::basic_outcomes::ABORT,
             yasmin_ros::basic_outcomes::ABORT},
            {yasmin_ros::basic_outcomes::CANCEL, "GO_ABOVE_DOCKING_STATION"},

        });

    sm->add_state(
        "CONVERGE_DOCKING_STATION",
        std::make_shared<ConvergeDockingStationState>(),
        {
            {yasmin_ros::basic_outcomes::SUCCEED,
             yasmin_ros::basic_outcomes::SUCCEED},
            {yasmin_ros::basic_outcomes::ABORT,
             yasmin_ros::basic_outcomes::ABORT},
            {yasmin_ros::basic_outcomes::CANCEL, "GO_ABOVE_DOCKING_STATION"},

        });
}

auto initialize_blackboard() {
    auto params = std::make_shared<rclcpp::Node>("dock_params");
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Creating params node");

    params->declare_parameter<double>("fsm.docking_station_offset");
    params->declare_parameter<int>("fsm.num_measurements");
    params->declare_parameter<std::string>("action_servers.reference_filter");
    params->declare_parameter<double>("fsm.dock_wait_time");

    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Parameters declared");

    auto blackboard = std::make_shared<yasmin::blackboard::Blackboard>();

    docking_fsm::PoseStamped dock_pose;
    docking_fsm::PoseStamped start_pose;
    blackboard->set<docking_fsm::PoseStamped>("dock_pose", dock_pose);
    blackboard->set<docking_fsm::PoseStamped>("start_pose", start_pose);

    blackboard->set<double>(
        "docking_station_offset",
        params->get_parameter("fsm.docking_station_offset").as_double());
    blackboard->set<bool>("return_home", false);
    blackboard->set<bool>("is_docked", false);
    blackboard->set<bool>("is_home", true);
    blackboard->set<bool>("is_error", false);
    blackboard->set<bool>("has_finished_converging", false);
    blackboard->set<int>(
        "num_measurements",
        params->get_parameter("fsm.num_measurements").as_int());
    blackboard->set<std::string>(
        "reference_filter_action",
        params->get_parameter("action_servers.reference_filter").as_string());
    blackboard->set<double>(
        "dock_wait_time",
        params->get_parameter("fsm.dock_wait_time").as_double());

    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Blackboard created");

    return blackboard;
}

int main(int argc, char* argv[]) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Docking");
    rclcpp::init(argc, argv);

    yasmin_ros::set_ros_loggers();

    std::shared_ptr<yasmin::StateMachine> sm = create_state_machines();
    std::shared_ptr<yasmin::StateMachine> nested_sm = create_state_machines();

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

    auto blackboard = initialize_blackboard();

    add_states(sm, nested_sm);
    add_states_nested(nested_sm, blackboard);

    yasmin_viewer::YasminViewerPub yasmin_pub("Docking", sm);
    yasmin_viewer::YasminViewerPub yasmin_pub_nested("DockingNested",
                                                     nested_sm);

    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "State machines created");

    try {
        std::string outcome = (*sm.get())(blackboard);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), outcome.c_str());
    } catch (const std::exception& e) {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), e.what());
        rcutils_reset_error();
    }

    if (!rclcpp::ok()) {
        RCLCPP_INFO(
            rclcpp::get_logger("rclcpp"),
            "ROS2 context is already invalid. Skipping publisher destruction.");
        return 1;
    }

    if (rclcpp::ok()) {
        sm.reset();
        blackboard.reset();

        rclcpp::shutdown();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "ROS2 shutdown completed gracefully.");
    }

    return 0;
}
