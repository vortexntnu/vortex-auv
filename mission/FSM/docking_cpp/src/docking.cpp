#include <docking_cpp/docking.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

FindDockState::FindDockState()
    : yasmin_ros::ActionState<FilteredPose>(
          "filtered_pose",
          std::bind(&FindDockState::create_goal_handler, this, _1),
          std::bind(&FindDockState::response_handler, this, _1, _2),
          std::bind(&FindDockState::print_feedback, this, _1, _2)) {};

FilteredPose::Goal FindDockState::create_goal_handler(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    auto goal = FilteredPose::Goal();
    goal.num_measurements = blackboard->get<bool>("num_measurements");

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal sent to action server:");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  Number of measurements: %d",
                goal.num_measurements);

    return goal;
}

std::string FindDockState::response_handler(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
    FilteredPose::Result::SharedPtr response) {
    blackboard->set<PoseStamped>("dock_pose", response->filtered_pose);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Response received from action server:");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "  Dock pose: x = %f, y = %f, z = %f\n",
                response->filtered_pose.pose.position.x,
                response->filtered_pose.pose.position.y,
                response->filtered_pose.pose.position.z);

    PoseStamped docking_offset_goal;
    docking_offset_goal = response->filtered_pose;
    docking_offset_goal.pose.position.z +=
        blackboard->get<double>("docking_station_offset");
    blackboard->set<PoseStamped>("docking_offset_goal", docking_offset_goal);

    return yasmin_ros::basic_outcomes::SUCCEED;
}

void FindDockState::print_feedback(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
    std::shared_ptr<const FilteredPose::Feedback> feedback) {
    blackboard->set<Pose>("current_pose", feedback->current_pose.pose);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Current position: x = %f, y = %f, z = %f\n",
                feedback->current_pose.pose.position.x,
                feedback->current_pose.pose.position.y,
                feedback->current_pose.pose.position.z);
}

GoToDockState::GoToDockState()
    : yasmin_ros::ActionState<LOSGuidance>(
          "/los_guidance",
          std::bind(&GoToDockState::create_goal_handler, this, _1),
          std::bind(&GoToDockState::response_handler, this, _1, _2),
          std::bind(&GoToDockState::print_feedback, this, _1, _2)) {};

LOSGuidance::Goal GoToDockState::create_goal_handler(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    auto goal = LOSGuidance::Goal();

    blackboard->set<bool>("is_home", false);

    PoseStamped docking_goal =
        blackboard->get<PoseStamped>("docking_offset_goal");

    PointStamped docking_point;
    docking_point.point = docking_goal.pose.position;
    docking_point.header = docking_goal.header;

    goal.goal = docking_point;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal sent to action server:");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "  Position: x = %f, y = %f, z = %f", docking_point.point.x,
                docking_point.point.y, docking_point.point.z);

    return goal;
}

std::string GoToDockState::response_handler(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
    LOSGuidance::Result::SharedPtr response) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Response received from action server:");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  Success: %s\n",
                response->success ? "true" : "false");

    blackboard->set<bool>("has_finished_converging", response->success);

    if (blackboard->get<bool>("has_finished_converging")) {
        return yasmin_ros::basic_outcomes::SUCCEED;
    } else {
        return yasmin_ros::basic_outcomes::ABORT;
    }
}

void GoToDockState::print_feedback(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
    std::shared_ptr<const LOSGuidance::Feedback> feedback) {
    blackboard->set<vortex_msgs::msg::LOSGuidance>("LOS_data",
                                                   feedback->feedback);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Current surge, pitch and yaw: %f, %f, %f",
                feedback->feedback.surge, feedback->feedback.pitch,
                feedback->feedback.yaw);
}

GoOverDockState::GoOverDockState()
    : yasmin_ros::ActionState<ReferenceFilterWaypoint>(
          "/reference_filter",
          std::bind(&GoOverDockState::create_goal_handler, this, _1),
          std::bind(&GoOverDockState::response_handler, this, _1, _2),
          std::bind(&GoOverDockState::print_feedback, this, _1, _2)) {};

ReferenceFilterWaypoint::Goal GoOverDockState::create_goal_handler(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    auto goal = ReferenceFilterWaypoint::Goal();

    auto docking_offset_goal =
        blackboard->get<PoseStamped>("docking_offset_goal");
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

std::string GoOverDockState::response_handler(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
    ReferenceFilterWaypoint::Result::SharedPtr response) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Response received from action server:");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  Success: %s\n",
                response->success ? "true" : "false");

    blackboard->set<bool>("is_docked", true);

    if (blackboard->get<bool>("is_docked")) {
        return yasmin_ros::basic_outcomes::SUCCEED;
    } else {
        return yasmin_ros::basic_outcomes::ABORT;
    }
}

void GoOverDockState::print_feedback(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
    std::shared_ptr<const ReferenceFilterWaypoint::Feedback> feedback) {
    Pose current_pose = Pose();
    current_pose.position.x = feedback->feedback.x;
    current_pose.position.y = feedback->feedback.y;
    current_pose.position.z = feedback->feedback.z;

    blackboard->set<Pose>("current_pose", current_pose);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Current position: x = %f, y = %f, z = %f\n",
                feedback->feedback.x, feedback->feedback.y,
                feedback->feedback.z);
}

std::string DockedState(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    // THIS IS NOT A GOOD SOLUTION
    int i = 0;
    // This code is a bad way to simulate waiting for 10 seconds
    while (i < 1000) {
        blackboard->set<bool>("is_docked", true);
        if (blackboard->get<bool>("return_home")) {
            return yasmin_ros::basic_outcomes::SUCCEED;
        }

        i++;
    };

    return yasmin_ros::basic_outcomes::ABORT;
};

ReturnHomeState::ReturnHomeState()
    : yasmin_ros::ActionState<ReferenceFilterWaypoint>(
          "/reference_filter",
          std::bind(&ReturnHomeState::create_goal_handler, this, _1),
          std::bind(&ReturnHomeState::response_handler, this, _1, _2),
          std::bind(&ReturnHomeState::print_feedback, this, _1, _2)) {};

ReferenceFilterWaypoint::Goal ReturnHomeState::create_goal_handler(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    auto goal = ReferenceFilterWaypoint::Goal();

    blackboard->set<bool>("is_docked", false);

    PoseStamped start_pose = blackboard->get<PoseStamped>("start_pose");

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
    ReferenceFilterWaypoint::Result::SharedPtr response) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Response received from action server:");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  Success: %s\n",
                response->success ? "true" : "false");

    blackboard->set<bool>("is_home", response->success);

    return yasmin_ros::basic_outcomes::SUCCEED;
}

void ReturnHomeState::print_feedback(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
    std::shared_ptr<const ReferenceFilterWaypoint::Feedback> feedback) {
    Pose current_pose = Pose();
    current_pose.position.x = feedback->feedback.x;
    current_pose.position.y = feedback->feedback.y;
    current_pose.position.z = feedback->feedback.z;
    current_pose.orientation.x = feedback->feedback.roll;
    current_pose.orientation.y = feedback->feedback.pitch;
    current_pose.orientation.z = feedback->feedback.yaw;

    blackboard->set<Pose>("current_pose", current_pose);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Current position: x = %f, y = %f, z = %f\n",
                current_pose.position.x, current_pose.position.y,
                current_pose.position.z);
}

AbortState::AbortState()
    : yasmin_ros::ActionState<ReferenceFilterWaypoint>(
          "/fsm/abort",
          std::bind(&AbortState::create_goal_handler, this, _1),
          std::bind(&AbortState::response_handler, this, _1, _2),
          std::bind(&AbortState::print_feedback, this, _1, _2)) {};

ReferenceFilterWaypoint::Goal AbortState::create_goal_handler(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    auto goal = ReferenceFilterWaypoint::Goal();

    PoseStamped start_pose = blackboard->get<PoseStamped>("start_pose");
    goal.goal = start_pose;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal sent to action server:");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "  Start position: x = %f, y = %f, z = %f\n",
                start_pose.pose.position.x, start_pose.pose.position.y,
                start_pose.pose.position.z);

    return goal;
}

std::string AbortState::response_handler(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
    ReferenceFilterWaypoint::Result::SharedPtr response) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Response received from action server:");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  Success: %s\n",
                response->success ? "true" : "false");

    blackboard->set<bool>("is_home", response->success);
    return yasmin_ros::basic_outcomes::SUCCEED;
}

void AbortState::print_feedback(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
    std::shared_ptr<const ReferenceFilterWaypoint::Feedback> feedback) {
    Pose current_pose = Pose();
    current_pose.position.x = feedback->feedback.x;
    current_pose.position.y = feedback->feedback.y;
    current_pose.position.z = feedback->feedback.z;
    current_pose.orientation.x = feedback->feedback.roll;
    current_pose.orientation.y = feedback->feedback.pitch;
    current_pose.orientation.z = feedback->feedback.yaw;

    blackboard->set<Pose>("current_pose", current_pose);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Current position: x = %f, y = %f, z = %f\n",
                current_pose.position.x, current_pose.position.y,
                current_pose.position.z);
}

std::string ErrorState(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    blackboard->set<bool>("is_error", true);
    return yasmin_ros::basic_outcomes::SUCCEED;
};

GoDownState::GoDownState()
    : yasmin_ros::ActionState<ReferenceFilterWaypoint>(
          "/reference_filter",
          std::bind(&GoDownState::create_goal_handler, this, _1),
          std::bind(&GoDownState::response_handler, this, _1, _2),
          std::bind(&GoDownState::print_feedback, this, _1, _2)) {};

ReferenceFilterWaypoint::Goal GoDownState::create_goal_handler(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    auto goal = ReferenceFilterWaypoint::Goal();

    PoseStamped dock_pose = blackboard->get<PoseStamped>("dock_pose");
    goal.goal = dock_pose;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal sent to action server:");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "  Position: x = %f, y = %f, z = %f\n",
                goal.goal.pose.position.x, goal.goal.pose.position.y,
                goal.goal.pose.position.z);

    return goal;
}

std::string GoDownState::response_handler(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
    ReferenceFilterWaypoint::Result::SharedPtr response) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Response received from action server:");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  Success: %s\n",
                response->success ? "true" : "false");

    blackboard->set<bool>("has_finished_converging", response->success);
    if (blackboard->get<bool>("has_finished_converging")) {
        return yasmin_ros::basic_outcomes::SUCCEED;
    } else {
        return yasmin_ros::basic_outcomes::ABORT;
    }
}

void GoDownState::print_feedback(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
    std::shared_ptr<const ReferenceFilterWaypoint::Feedback> feedback) {
    Pose current_pose = Pose();
    current_pose.position.x = feedback->feedback.x;
    current_pose.position.y = feedback->feedback.y;
    current_pose.position.z = feedback->feedback.z;

    blackboard->set<Pose>("current_pose", current_pose);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Current position: x = %f, y = %f, z = %f\n",
                feedback->feedback.x, feedback->feedback.y,
                feedback->feedback.z);
}

std::shared_ptr<yasmin::StateMachine> create_state_machines() {
    auto sm = std::make_shared<yasmin::StateMachine>(
        std::initializer_list<std::string>{
            "error",
            yasmin_ros::basic_outcomes::SUCCEED,
            yasmin_ros::basic_outcomes::CANCEL,
            yasmin_ros::basic_outcomes::ABORT,
            yasmin_ros::basic_outcomes::TIMEOUT,
        });
    return sm;
}
std::shared_ptr<yasmin::StateMachine> create_state_machines_nested() {
    auto sm = std::make_shared<yasmin::StateMachine>(
        std::initializer_list<std::string>{
            "error",
            yasmin_ros::basic_outcomes::SUCCEED,
            yasmin_ros::basic_outcomes::CANCEL,
            yasmin_ros::basic_outcomes::ABORT,
            yasmin_ros::basic_outcomes::TIMEOUT,
        });
    return sm;
}

void add_states(std::shared_ptr<yasmin::StateMachine> sm,
                std::shared_ptr<yasmin::StateMachine> nested_sm) {
    sm->add_state("FIND_DOCK", std::make_shared<FindDockState>(),
                  {
                      {yasmin_ros::basic_outcomes::SUCCEED, "GO_TO_DOCK"},
                      {yasmin_ros::basic_outcomes::CANCEL, "error"},
                      {yasmin_ros::basic_outcomes::ABORT, "ABORT"},
                  });
    sm->add_state("GO_TO_DOCK", std::make_shared<GoToDockState>(),
                  {
                      {yasmin_ros::basic_outcomes::SUCCEED, "dock_fsm"},
                      {yasmin_ros::basic_outcomes::CANCEL, "error"},
                      {yasmin_ros::basic_outcomes::ABORT, "ABORT"},
                  });

    sm->add_state("DOCKED",
                  std::make_shared<yasmin::CbState>(
                      std::initializer_list<std::string>{
                          "error", yasmin_ros::basic_outcomes::SUCCEED,
                          yasmin_ros::basic_outcomes::CANCEL,
                          yasmin_ros::basic_outcomes::ABORT},
                      DockedState),
                  {
                      {yasmin_ros::basic_outcomes::SUCCEED,
                       yasmin_ros::basic_outcomes::SUCCEED},
                      {yasmin_ros::basic_outcomes::CANCEL, "RETURN_HOME"},
                      {yasmin_ros::basic_outcomes::ABORT, "ABORT"},

                  });
    sm->add_state("RETURN_HOME", std::make_shared<ReturnHomeState>(),
                  {
                      {yasmin_ros::basic_outcomes::SUCCEED, "FIND_DOCK"},
                      {yasmin_ros::basic_outcomes::CANCEL, "error"},
                      {yasmin_ros::basic_outcomes::ABORT, "ABORT"},

                  });
    sm->add_state("ABORT", std::make_shared<AbortState>(),
                  {
                      {yasmin_ros::basic_outcomes::SUCCEED, "FIND_DOCK"},
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
    sm->add_state("dock_fsm", nested_sm,
                  {
                      {yasmin_ros::basic_outcomes::SUCCEED, "DOCKED"},
                      {yasmin_ros::basic_outcomes::ABORT, "ABORT"},

                  });
}

void add_states_nested(std::shared_ptr<yasmin::StateMachine> sm) {
    sm->add_state("GO_OVER_DOCK", std::make_shared<GoOverDockState>(),
                  {
                      {yasmin_ros::basic_outcomes::SUCCEED, "GO_DOWN_DOCK"},
                      {yasmin_ros::basic_outcomes::ABORT,
                       yasmin_ros::basic_outcomes::ABORT},

                  });

    sm->add_state("GO_DOWN_DOCK", std::make_shared<GoDownState>(),
                  {
                      {yasmin_ros::basic_outcomes::SUCCEED,
                       yasmin_ros::basic_outcomes::SUCCEED},
                      {yasmin_ros::basic_outcomes::ABORT,
                       yasmin_ros::basic_outcomes::ABORT},

                  });
}

auto initialize_blackboard() {
    auto params = std::make_shared<rclcpp::Node>("dock_params");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Creating params node");

    params->declare_parameter<double>("docking_station_offset");
    params->declare_parameter<bool>("return_home");
    params->declare_parameter<bool>("is_docked");
    params->declare_parameter<bool>("is_home");
    params->declare_parameter<bool>("is_error");
    params->declare_parameter<bool>("has_finished_converging");
    params->declare_parameter<int>("num_measurements");

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Parameters declared");

    auto blackboard = std::make_shared<yasmin::blackboard::Blackboard>();

    PoseStamped dock_pose;
    PoseStamped start_pose;
    blackboard->set<PoseStamped>("dock_pose", dock_pose);
    blackboard->set<PoseStamped>("start_pose", start_pose);

    blackboard->set<double>(
        "docking_station_offset",
        params->get_parameter("docking_station_offset").as_double());
    blackboard->set<bool>("return_home",
                          params->get_parameter("return_home").as_bool());
    blackboard->set<bool>("is_docked",
                          params->get_parameter("is_docked").as_bool());
    blackboard->set<bool>("is_home",
                          params->get_parameter("is_home").as_bool());
    blackboard->set<bool>("is_error",
                          params->get_parameter("is_error").as_bool());
    blackboard->set<bool>(
        "has_finished_converging",
        params->get_parameter("has_finished_converging").as_bool());
    blackboard->set<int>("num_measurements",
                         params->get_parameter("num_measurements").as_int());

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Blackboard created");

    return blackboard;
}

int main(int argc, char* argv[]) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "docking");
    rclcpp::init(argc, argv);

    yasmin_ros::set_ros_loggers();

    std::shared_ptr<yasmin::StateMachine> sm = create_state_machines();
    std::shared_ptr<yasmin::StateMachine> nested_sm =
        create_state_machines_nested();

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

    add_states(sm, nested_sm);
    add_states_nested(nested_sm);

    yasmin_viewer::YasminViewerPub yasmin_pub("Docking", sm);
    yasmin_viewer::YasminViewerPub yasmin_pub_nested("DockingNested",
                                                     nested_sm);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "State machines created");

    auto blackboard = initialize_blackboard();

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
        return 1;  // Exit with error
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
