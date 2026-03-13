#include "pipeline/pipeline.hpp"

#include <chrono>
#include <cmath>
#include <thread>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

SearchState::SearchState(yasmin::Blackboard::SharedPtr blackboard)
    : yasmin_ros::ActionState<pipeline_fsm::LandmarkPollingAction>(
          blackboard->get<std::string>("landmark_polling_action"),
          std::bind(&SearchState::create_goal_handler, this, _1),
          yasmin::Outcomes{"landmark_found", yasmin_ros::basic_outcomes::ABORT,
                           yasmin_ros::basic_outcomes::CANCEL},
          std::bind(&SearchState::response_handler, this, _1, _2)) {
    auto node = yasmin_ros::YasminNode::get_instance();
    wm_client_ =
        rclcpp_action::create_client<pipeline_fsm::WaypointManagerAction>(
            node, blackboard->get<std::string>("waypoint_manager_action"));
    wp_service_client_ = node->create_client<pipeline_fsm::SendWaypointsSrv>(
        "waypoint_addition");
}

std::string SearchState::execute(yasmin::Blackboard::SharedPtr blackboard) {
    auto wm_handle = start_waypoint_manager(blackboard);
    if (!wm_handle) {
        return yasmin_ros::basic_outcomes::ABORT;
    }

    std::string outcome =
        yasmin_ros::ActionState<pipeline_fsm::LandmarkPollingAction>::execute(
            blackboard);

    stop_waypoint_manager(wm_handle);

    return outcome;
}

pipeline_fsm::WaypointManagerGoalHandle::SharedPtr
SearchState::start_waypoint_manager(yasmin::Blackboard::SharedPtr blackboard) {
    YASMIN_LOG_INFO("SearchState: starting WaypointManager in persistent mode");

    if (!wm_client_->wait_for_action_server(5s)) {
        YASMIN_LOG_ERROR("SearchState: WaypointManager server not available");
        return nullptr;
    }

    pipeline_fsm::WaypointManagerAction::Goal goal;
    goal.persistent = true;
    goal.convergence_threshold =
        blackboard->get<double>("search_convergence_threshold");
    // Waypoints intentionally empty.
    // push_initial_waypoint() in create_goal_handler(), and external
    auto goal_future = wm_client_->async_send_goal(goal);
    if (goal_future.wait_for(5s) != std::future_status::ready) {
        YASMIN_LOG_ERROR(
            "SearchState: WaypointManager goal acceptance timed out");
        return nullptr;
    }

    auto handle = goal_future.get();
    if (!handle) {
        YASMIN_LOG_ERROR("SearchState: WaypointManager goal rejected");
        return nullptr;
    }

    YASMIN_LOG_INFO("SearchState: WaypointManager running");
    return handle;
}

void SearchState::stop_waypoint_manager(
    pipeline_fsm::WaypointManagerGoalHandle::SharedPtr handle) {
    if (!handle)
        return;

    YASMIN_LOG_INFO("SearchState: stopping WaypointManager");

    // Must use the same client that sent the goal.
    auto cancel_future = wm_client_->async_cancel_goal(handle);
    if (cancel_future.wait_for(5s) != std::future_status::ready) {
        YASMIN_LOG_WARN("SearchState: WaypointManager cancel timed out");
    } else {
        YASMIN_LOG_INFO("SearchState: WaypointManager stopped");
    }
}

pipeline_fsm::LandmarkPollingAction::Goal SearchState::create_goal_handler(
    yasmin::Blackboard::SharedPtr blackboard) {
    // Push initial waypoint so drone starts moving immediately.
    push_initial_waypoint(blackboard);

    pipeline_fsm::LandmarkPollingAction::Goal goal;
    goal.type.value = blackboard->get<int8_t>("landmark_type");
    goal.subtype.value = blackboard->get<int8_t>("landmark_subtype");
    YASMIN_LOG_INFO("SearchState: polling for landmark type=%d subtype=%d",
                    goal.type.value, goal.subtype.value);
    return goal;
}

std::string SearchState::response_handler(
    yasmin::Blackboard::SharedPtr blackboard,
    pipeline_fsm::LandmarkPollingAction::Result::SharedPtr response) {
    if (!response->landmarks.landmarks.empty()) {
        blackboard->set<vortex_msgs::msg::Landmark>(
            "found_landmark", response->landmarks.landmarks[0]);
        YASMIN_LOG_INFO("SearchState: landmark found!");
        return "landmark_found";
    }
    YASMIN_LOG_WARN("SearchState: polling server returned empty result");
    return yasmin_ros::basic_outcomes::ABORT;
}

void SearchState::push_initial_waypoint(
    yasmin::Blackboard::SharedPtr blackboard) {
    if (!wp_service_client_->wait_for_service(3s)) {
        YASMIN_LOG_WARN("SearchState: waypoint_addition service not available");
        return;
    }

    auto req = std::make_shared<pipeline_fsm::SendWaypointsSrv::Request>();
    req->waypoints = blackboard->get<std::vector<vortex_msgs::msg::Waypoint>>(
        "search_waypoints");
    req->overwrite_prior_waypoints = true;
    req->take_priority = false;

    wp_service_client_->async_send_request(
        req, [](rclcpp::Client<pipeline_fsm::SendWaypointsSrv>::SharedFuture
                    future) {
            if (!future.get()->success) {
                YASMIN_LOG_WARN(
                    "SearchState: waypoint_addition service call failed");
            } else {
                YASMIN_LOG_INFO(
                    "SearchState: initial search waypoint sent to "
                    "WaypointManager");
            }
        });
}

// LandmarkConvergeState

LandmarkConvergeState::LandmarkConvergeState(
    yasmin::Blackboard::SharedPtr blackboard)
    : yasmin_ros::ActionState<pipeline_fsm::LandmarkConvergenceAction>(
          blackboard->get<std::string>("landmark_convergence_action"),
          std::bind(&LandmarkConvergeState::create_goal_handler, this, _1)) {}

pipeline_fsm::LandmarkConvergenceAction::Goal
LandmarkConvergeState::create_goal_handler(
    yasmin::Blackboard::SharedPtr blackboard) {
    auto goal = pipeline_fsm::LandmarkConvergenceAction::Goal();
    auto found_landmark =
        blackboard->get<vortex_msgs::msg::Landmark>("found_landmark");
    goal.type.value = found_landmark.type.value;
    goal.subtype.value = found_landmark.subtype.value;
    goal.convergence_offset =
        blackboard->get<geometry_msgs::msg::Pose>("convergence_offset");
    goal.convergence_threshold =
        blackboard->get<double>("convergence_threshold");
    goal.dead_reckoning_threshold =
        blackboard->get<double>("dead_reckoning_threshold");
    goal.track_loss_timeout_sec =
        blackboard->get<double>("track_loss_timeout_sec");
    YASMIN_LOG_INFO("LandmarkConvergeState: converging on type=%d subtype=%d",
                    goal.type.value, goal.subtype.value);
    return goal;
}

// Blackboard initialisation

std::shared_ptr<yasmin::Blackboard> initialize_blackboard() {
    auto params = std::make_shared<rclcpp::Node>("pipeline_params");

    params->declare_parameter<std::string>("action_servers.waypoint_manager");
    params->declare_parameter<std::string>("action_servers.landmark_polling");
    params->declare_parameter<std::string>(
        "action_servers.landmark_convergence");
    params->declare_parameter<std::string>("topics.odom");
    params->declare_parameter<int>("fsm.pipeline.landmark_type");
    params->declare_parameter<int>("fsm.pipeline.landmark_subtype");
    params->declare_parameter<double>(
        "fsm.pipeline.search_convergence_threshold");
    params->declare_parameter<double>("fsm.pipeline.convergence_threshold");
    params->declare_parameter<double>("fsm.pipeline.dead_reckoning_threshold");
    params->declare_parameter<double>("fsm.pipeline.track_loss_timeout_sec");

    auto bb = std::make_shared<yasmin::Blackboard>();

    bb->set<std::string>(
        "waypoint_manager_action",
        params->get_parameter("action_servers.waypoint_manager").as_string());
    bb->set<std::string>(
        "landmark_polling_action",
        params->get_parameter("action_servers.landmark_polling").as_string());
    bb->set<std::string>(
        "landmark_convergence_action",
        params->get_parameter("action_servers.landmark_convergence")
            .as_string());
    bb->set<int8_t>(
        "landmark_type",
        static_cast<int8_t>(
            params->get_parameter("fsm.pipeline.landmark_type").as_int()));
    bb->set<int8_t>(
        "landmark_subtype",
        static_cast<int8_t>(
            params->get_parameter("fsm.pipeline.landmark_subtype").as_int()));
    bb->set<double>(
        "search_convergence_threshold",
        params->get_parameter("fsm.pipeline.search_convergence_threshold")
            .as_double());
    bb->set<double>("convergence_threshold",
                    params->get_parameter("fsm.pipeline.convergence_threshold")
                        .as_double());
    bb->set<double>(
        "dead_reckoning_threshold",
        params->get_parameter("fsm.pipeline.dead_reckoning_threshold")
            .as_double());
    bb->set<double>("track_loss_timeout_sec",
                    params->get_parameter("fsm.pipeline.track_loss_timeout_sec")
                        .as_double());

    auto odom_node = std::make_shared<rclcpp::Node>("pipeline_odom_reader");
    geometry_msgs::msg::Pose start_pose;
    start_pose.orientation.w = 1.0;
    bool odom_received = false;

    auto sub = odom_node->create_subscription<nav_msgs::msg::Odometry>(
        params->get_parameter("topics.odom").as_string(),
        rclcpp::QoS(1).best_effort(),
        [&](nav_msgs::msg::Odometry::SharedPtr msg) {
            start_pose = msg->pose.pose;
            odom_received = true;
        });

    auto deadline = odom_node->now() + rclcpp::Duration::from_seconds(5.0);
    while (!odom_received && rclcpp::ok() && odom_node->now() < deadline) {
        rclcpp::spin_some(odom_node);
        std::this_thread::sleep_for(50ms);
    }
    if (!odom_received) {
        YASMIN_LOG_WARN(
            "pipeline: no odom received — using origin as start pose");
    }

    const auto& q = start_pose.orientation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    vortex_msgs::msg::Waypoint wp;
    wp.pose.position.x = start_pose.position.x + 20.0 * std::cos(yaw);
    wp.pose.position.y = start_pose.position.y + 20.0 * std::sin(yaw);
    wp.pose.position.z = start_pose.position.z;
    wp.pose.orientation = start_pose.orientation;
    wp.mode = vortex_msgs::msg::Waypoint::ONLY_POSITION;

    bb->set<std::vector<vortex_msgs::msg::Waypoint>>("search_waypoints", {wp});

    geometry_msgs::msg::Pose no_offset;
    no_offset.orientation.w = 1.0;
    bb->set<geometry_msgs::msg::Pose>("convergence_offset", no_offset);

    return bb;
}

int main(int argc, char* argv[]) {
    YASMIN_LOG_INFO("Pipeline FSM starting");
    rclcpp::init(argc, argv);
    yasmin_ros::set_ros_loggers();

    auto blackboard = initialize_blackboard();

    auto sm = std::make_shared<yasmin::StateMachine>(
        std::set<std::string>{yasmin_ros::basic_outcomes::SUCCEED,
                              yasmin_ros::basic_outcomes::ABORT});

    rclcpp::on_shutdown([sm]() {
        if (sm->is_running())
            sm->cancel_state();
    });

    // SEARCH → CONVERGE
    sm->add_state("SEARCH", std::make_shared<SearchState>(blackboard),
                  {
                      {"landmark_found", "CONVERGE"},
                      {yasmin_ros::basic_outcomes::ABORT,
                       yasmin_ros::basic_outcomes::ABORT},
                      {yasmin_ros::basic_outcomes::CANCEL,
                       yasmin_ros::basic_outcomes::ABORT},
                  });

    sm->add_state("CONVERGE",
                  std::make_shared<LandmarkConvergeState>(blackboard),
                  {
                      {yasmin_ros::basic_outcomes::SUCCEED,
                       yasmin_ros::basic_outcomes::SUCCEED},
                      {yasmin_ros::basic_outcomes::ABORT,
                       yasmin_ros::basic_outcomes::ABORT},
                      {yasmin_ros::basic_outcomes::CANCEL,
                       yasmin_ros::basic_outcomes::ABORT},
                  });

    yasmin_viewer::YasminViewerPub yasmin_pub(sm, "PIPELINE_FSM");

    try {
        std::string outcome = (*sm)(blackboard);
        YASMIN_LOG_INFO("Pipeline FSM finished with outcome: %s",
                        outcome.c_str());
    } catch (const std::exception& e) {
        YASMIN_LOG_WARN(e.what());
        rcutils_reset_error();
    }

    if (rclcpp::ok()) {
        sm.reset();
        blackboard.reset();
        rclcpp::shutdown();
    }

    return 0;
}
