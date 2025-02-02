#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include <vortex_msgs/action/go_to_waypoint.hpp>
#include <vortex_msgs/action/locate_dock.hpp>
#include <vortex_msgs/action/navigate_waypoints.hpp>
#include <vortex_msgs/action/reference_filter_waypoint.hpp>

#include <std_msgs/msg/string.hpp>
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
using LocateDock = vortex_msgs::action::LocateDock;
using GoToWaypoint = vortex_msgs::action::GoToWaypoint;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using Pose = geometry_msgs::msg::Pose;
using ReferenceFilterWaypoint = vortex_msgs::action::ReferenceFilterWaypoint;
using ReferenceFilter = vortex_msgs::msg::ReferenceFilter;
using NavigateWaypoints = vortex_msgs::action::NavigateWaypoints;
using namespace yasmin;

class FindDockState : public yasmin_ros::ActionState<LocateDock> {
   public:
    FindDockState()
        : yasmin_ros::ActionState<LocateDock>(
              "search_dock",
              std::bind(&FindDockState::create_goal_handler, this, _1),
              std::bind(&FindDockState::response_handler, this, _1, _2),
              std::bind(&FindDockState::print_feedback, this, _1, _2)) {};

    LocateDock::Goal create_goal_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
        auto goal = LocateDock::Goal();
        goal.start_search = blackboard->get<bool>("start_search");

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "Goal sent to action server: \n");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  Start search: %s\n",
                    goal.start_search ? "true" : "false");

        return goal;
    }

    std::string response_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        LocateDock::Result::SharedPtr response) {
        blackboard->set<PoseStamped>("dock_pose", response->board_pose);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "Response received from action server: \n");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "  Dock pose: x = %f, y = %f, z = %f\n",
                    response->board_pose.pose.position.x,
                    response->board_pose.pose.position.y,
                    response->board_pose.pose.position.z);

        PoseStamped docking_offset_goal;
        docking_offset_goal = response->board_pose;
        docking_offset_goal.pose.position.z +=
            blackboard->get<float>("docking_station_offset");
        blackboard->set<PoseStamped>("docking_offset_goal",
                                     docking_offset_goal);

        return yasmin_ros::basic_outcomes::SUCCEED;
    }

    void print_feedback(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        std::shared_ptr<const LocateDock::Feedback> feedback) {
        blackboard->set<float>("confimred_search", feedback->confirmed);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Confirmed search: %i\n",
                    feedback->confirmed);
    }
};

class GoToDockState : public yasmin_ros::ActionState<NavigateWaypoints> {
   public:
    GoToDockState()
        : yasmin_ros::ActionState<NavigateWaypoints>(
              "/navigate_waypoints",
              std::bind(&GoToDockState::create_goal_handler, this, _1),
              std::bind(&GoToDockState::response_handler, this, _1, _2),
              std::bind(&GoToDockState::print_feedback, this, _1, _2)) {};

    NavigateWaypoints::Goal create_goal_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
        auto goal = NavigateWaypoints::Goal();

        blackboard->set<bool>("is_home", false);

        PoseStamped docking_goal =
            blackboard->get<PoseStamped>("docking_offset_goal");
        goal.waypoints.push_back(docking_goal);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "Goal sent to action server: \n");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "  Position: x = %f, y = %f, z = %f\n",
                    docking_goal.pose.position.x, docking_goal.pose.position.y,
                    docking_goal.pose.position.z);
        fprintf(
            stderr, "  Orientation: x = %f, y = %f, z = %f, w = %f\n",
            docking_goal.pose.orientation.x, docking_goal.pose.orientation.y,
            docking_goal.pose.orientation.z, docking_goal.pose.orientation.w);

        return goal;
    }

    std::string response_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        NavigateWaypoints::Result::SharedPtr response) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "Response received from action server: \n");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  Success: %s\n",
                    response->success ? "true" : "false");

        blackboard->set<bool>("has_finished_converging", response->success);

        if (blackboard->get<bool>("has_finished_converging")) {
            return yasmin_ros::basic_outcomes::SUCCEED;
        } else {
            return yasmin_ros::basic_outcomes::ABORT;
        }
    }

    void print_feedback(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        std::shared_ptr<const NavigateWaypoints::Feedback> feedback) {
        blackboard->set<Pose>("current_pose", feedback->current_pose);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "Current position: x = %f, y = %f, z = %f\n",
                    feedback->current_pose.position.x,
                    feedback->current_pose.position.y,
                    feedback->current_pose.position.z);
    }
};

class GoOverDockState
    : public yasmin_ros::ActionState<ReferenceFilterWaypoint> {
   public:
    GoOverDockState()
        : yasmin_ros::ActionState<ReferenceFilterWaypoint>(
              "/reference_filter",
              std::bind(&GoOverDockState::create_goal_handler, this, _1),
              std::bind(&GoOverDockState::response_handler, this, _1, _2),
              std::bind(&GoOverDockState::print_feedback, this, _1, _2)) {};

    ReferenceFilterWaypoint::Goal create_goal_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
        auto goal = ReferenceFilterWaypoint::Goal();

        auto docking_offset_goal =
            blackboard->get<PoseStamped>("docking_offset_goal");
        goal.goal = docking_offset_goal;

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "Goal sent to action server: \n");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "  Position: x = %f, y = %f, z = %f\n",
                    docking_offset_goal.pose.position.x,
                    docking_offset_goal.pose.position.y,
                    docking_offset_goal.pose.position.z);
        fprintf(stderr, "  Orientation: x = %f, y = %f, z = %f, w = %f\n",
                docking_offset_goal.pose.orientation.x,
                docking_offset_goal.pose.orientation.y,
                docking_offset_goal.pose.orientation.z,
                docking_offset_goal.pose.orientation.w);

        return goal;
    }

    std::string response_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        ReferenceFilterWaypoint::Result::SharedPtr response) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "Response received from action server: \n");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  Success: %s\n",
                    response->success ? "true" : "false");

        blackboard->set<bool>("is_docked", true);

        if (blackboard->get<bool>("is_docked")) {
            return yasmin_ros::basic_outcomes::SUCCEED;
        } else {
            return yasmin_ros::basic_outcomes::ABORT;
        }
    }

    void print_feedback(
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
};

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

class ReturnHomeState : public yasmin_ros::ActionState<NavigateWaypoints> {
   public:
    ReturnHomeState()
        : yasmin_ros::ActionState<NavigateWaypoints>(
              "/fsm/return_home",
              std::bind(&ReturnHomeState::create_goal_handler, this, _1),
              std::bind(&ReturnHomeState::response_handler, this, _1, _2),
              std::bind(&ReturnHomeState::print_feedback, this, _1, _2)) {};

    NavigateWaypoints::Goal create_goal_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
        auto goal = NavigateWaypoints::Goal();

        blackboard->set<bool>("is_docked", false);

        PoseStamped docking_goal =
            blackboard->get<PoseStamped>("docking_offset_goal");
        PoseStamped start_pose = blackboard->get<PoseStamped>("start_pose");

        goal.waypoints.push_back(docking_goal);
        goal.waypoints.push_back(start_pose);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "Goal sent to action server: \n");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "  Docking position: x = %f, y = %f, z = %f\n",
                    docking_goal.pose.position.x, docking_goal.pose.position.y,
                    docking_goal.pose.position.z);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "  Start position: x = %f, y = %f, z = %f\n",
                    start_pose.pose.position.x, start_pose.pose.position.y,
                    start_pose.pose.position.z);

        return goal;
    }

    std::string response_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        NavigateWaypoints::Result::SharedPtr response) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "Response received from action server: \n");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  Success: %s\n",
                    response->success ? "true" : "false");

        blackboard->set<bool>("is_home", response->success);

        return yasmin_ros::basic_outcomes::SUCCEED;
    }

    void print_feedback(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        std::shared_ptr<const NavigateWaypoints::Feedback> feedback) {
        blackboard->set<Pose>("current_pose", feedback->current_pose);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "Current position: x = %f, y = %f, z = %f\n",
                    feedback->current_pose.position.x,
                    feedback->current_pose.position.y,
                    feedback->current_pose.position.z);
    }
};

class AbortState : public yasmin_ros::ActionState<NavigateWaypoints> {
   public:
    AbortState()
        : yasmin_ros::ActionState<NavigateWaypoints>(
              "/fsm/abort",
              std::bind(&AbortState::create_goal_handler, this, _1),
              std::bind(&AbortState::response_handler, this, _1, _2),
              std::bind(&AbortState::print_feedback, this, _1, _2)) {};

    NavigateWaypoints::Goal create_goal_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
        auto goal = NavigateWaypoints::Goal();

        PoseStamped start_pose = blackboard->get<PoseStamped>("start_pose");
        goal.waypoints.push_back(start_pose);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "Goal sent to action server: \n");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "  Start position: x = %f, y = %f, z = %f\n",
                    start_pose.pose.position.x, start_pose.pose.position.y,
                    start_pose.pose.position.z);

        return goal;
    }

    std::string response_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        NavigateWaypoints::Result::SharedPtr response) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "Response received from action server: \n");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  Success: %s\n",
                    response->success ? "true" : "false");

        blackboard->set<bool>("is_home", response->success);
        return yasmin_ros::basic_outcomes::SUCCEED;
    }

    void print_feedback(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        std::shared_ptr<const NavigateWaypoints::Feedback> feedback) {
        blackboard->set<Pose>("current_pose", feedback->current_pose);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "Current position: x = %f, y = %f, z = %f\n",
                    feedback->current_pose.position.x,
                    feedback->current_pose.position.y,
                    feedback->current_pose.position.z);
    }
};

std::string ErrorState(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    blackboard->set<bool>("is_error", true);
    return yasmin_ros::basic_outcomes::SUCCEED;
};

class GoDownState : public yasmin_ros::ActionState<ReferenceFilterWaypoint> {
   public:
    GoDownState()
        : yasmin_ros::ActionState<ReferenceFilterWaypoint>(
              "/reference_filter",
              std::bind(&GoDownState::create_goal_handler, this, _1),
              std::bind(&GoDownState::response_handler, this, _1, _2),
              std::bind(&GoDownState::print_feedback, this, _1, _2)) {};

    ReferenceFilterWaypoint::Goal create_goal_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
        auto goal = ReferenceFilterWaypoint::Goal();

        PoseStamped dock_pose = blackboard->get<PoseStamped>("dock_pose");
        goal.goal = dock_pose;

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "Goal sent to action server: \n");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "  Position: x = %f, y = %f, z = %f\n",
                    goal.goal.pose.position.x, goal.goal.pose.position.y,
                    goal.goal.pose.position.z);

        return goal;
    }

    std::string response_handler(
        std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
        ReferenceFilterWaypoint::Result::SharedPtr response) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "Response received from action server: \n");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  Success: %s\n",
                    response->success ? "true" : "false");

        blackboard->set<bool>("has_finished_converging", response->success);
        if (blackboard->get<bool>("has_finished_converging")) {
            return yasmin_ros::basic_outcomes::SUCCEED;
        } else {
            return yasmin_ros::basic_outcomes::ABORT;
        }
    }

    void print_feedback(
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
};

// Create state machines
std::shared_ptr<yasmin::StateMachine> create_state_machines() {
    auto sm = std::make_shared<yasmin::StateMachine>(
        std::initializer_list<std::string>{
            "error",
            yasmin_ros::basic_outcomes::SUCCEED,
            yasmin_ros::basic_outcomes::CANCEL,
            yasmin_ros::basic_outcomes::ABORT,
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
        });
    return sm;
}

// Add states
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
                      {yasmin_ros::basic_outcomes::CANCEL, "aborted"},
                      {yasmin_ros::basic_outcomes::ABORT, "aborted"},

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
                      {yasmin_ros::basic_outcomes::ABORT, "aborted"},

                  });

    sm->add_state("GO_DOWN_DOCK", std::make_shared<GoDownState>(),
                  {
                      {yasmin_ros::basic_outcomes::SUCCEED,
                       yasmin_ros::basic_outcomes::SUCCEED},
                      {yasmin_ros::basic_outcomes::ABORT, "aborted"},

                  });
}

class MakeBlackboard : public rclcpp::Node {
   public:
    MakeBlackboard() : Node("make_blackboard") {
        RCLCPP_INFO(this->get_logger(), "Creating MakeBlackboard node");
        // Initialize the blackboard
        this->blackboards = std::make_shared<yasmin::blackboard::Blackboard>();

        // Declare parameters with a map
        this->declare_parameters(
            "",  // Namespace
            std::map<std::string, rclcpp::ParameterValue>{
                {"dock_pose", rclcpp::ParameterValue(0)},
                {"start_pose", rclcpp::ParameterValue(0)},
                {"docking_station_offset", rclcpp::ParameterValue(0.0)},
                {"return_home", rclcpp::ParameterValue(false)},
                {"is_docked", rclcpp::ParameterValue(false)},
                {"is_home", rclcpp::ParameterValue(false)},
                {"is_error", rclcpp::ParameterValue(false)},
                {"has_finished_converging", rclcpp::ParameterValue(false)},
                {"start_search", rclcpp::ParameterValue(true)},
            });
        RCLCPP_INFO(this->get_logger(), "Parameters declared");
        bool start_search;  // Create a local variable to hold the value
        if (this->get_parameter("start_search",
                                start_search)) {  // Check if the parameter
                                                  // exists and was retrieved
            this->blackboards->set<bool>(
                "start_search", start_search);  // Set it in the blackboard
            RCLCPP_INFO(this->get_logger(),
                        "Parameter 'start_search' set to %d", start_search);
        } else {
            RCLCPP_WARN(this->get_logger(),
                        "Parameter 'start_search' not set or invalid");
        }
    }

    // Getter for the blackboard
    std::shared_ptr<yasmin::blackboard::Blackboard> get_blackboard() const {
        return this->blackboards;
    }

   private:
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboards;
};

int main(int argc, char* argv[]) {
    YASMIN_LOG_INFO("docking");
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

    auto CreateBlackboard = std::make_shared<MakeBlackboard>();
    auto blackboard = CreateBlackboard->get_blackboard();

    // ******************* THIS VALUE CAN BE CHANGED ***************
    double docking_station_offset = -1.0;
    blackboard->set<float>("docking_station_offset", docking_station_offset);

    //**************************************************************

    // ****************** TEST VALUES*******************************
    PoseStamped dock_pose;
    dock_pose.pose.position.x = 0.0;
    dock_pose.pose.position.y = 7.5;
    dock_pose.pose.position.z = 3.638;
    dock_pose.pose.orientation.x = 0.0;
    dock_pose.pose.orientation.y = 0.0;
    dock_pose.pose.orientation.z = 0.0;
    dock_pose.pose.orientation.w = 1.0;

    PoseStamped start_pose;
    start_pose.pose.position.x = 0.0;
    start_pose.pose.position.y = 0.0;
    start_pose.pose.position.z = 0.0;

    blackboard->set<bool>("start_search", true);
    blackboard->set<PoseStamped>("current_pose", PoseStamped());
    blackboard->set<PoseStamped>("start_pose", start_pose);
    blackboard->set<PoseStamped>("dock_pose", dock_pose);
    blackboard->set<bool>("return_home", true);
    blackboard->set<bool>("is_docked", false);
    blackboard->set<bool>("is_home", false);
    blackboard->set<bool>("is_error", false);
    blackboard->set<bool>("has_finished_converging", false);

    // *************************************************************

    try {
        std::string outcome = (*sm.get())(blackboard);
        YASMIN_LOG_INFO(outcome.c_str());
    } catch (const std::exception& e) {
        YASMIN_LOG_WARN(e.what());
        rcutils_reset_error();
    }

    if (!rclcpp::ok()) {
        YASMIN_LOG_WARN(
            "ROS2 context is already invalid. Skipping publisher destruction.");
        return 1;  // DONT KNOW THE CORRECT RETURN VALUE
    }

    if (rclcpp::ok()) {
        sm.reset();
        blackboard.reset();

        rclcpp::shutdown();
        YASMIN_LOG_INFO("ROS2 shutdown completed gracefully.");
    }

    return 0;
}
