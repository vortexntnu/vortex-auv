#include "waypoint_manager/waypoint_manager.hpp"
#include <functional>
#include <memory>

WaypointManagerNode::WaypointManagerNode() : Node("waypoint_manager_node") {
    // Create callback groups to allow concurrent execution
    auto server_cb_group =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    auto client_cb_group =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    setup_action_server();
    setup_action_client();
    setup_subscriber();

    spdlog::info("Waypoint Manager Node initialized");
}

void WaypointManagerNode::setup_action_server() {
    this->declare_parameter<std::string>("action_servers.waypoint_manager",
                                         "waypoint_manager");
    std::string action_server_name =
        this->get_parameter("action_servers.waypoint_manager").as_string();

    rclcpp::SubscriptionOptions server_options;
    server_options.callback_group =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    action_server_ = rclcpp_action::create_server<WaypointManagerAction>(
        this, action_server_name,
        std::bind(&WaypointManagerNode::handle_goal, this,
                  std::placeholders::_1, std::placeholders::_2),
        std::bind(&WaypointManagerNode::handle_cancel, this,
                  std::placeholders::_1),
        std::bind(&WaypointManagerNode::handle_accepted, this,
                  std::placeholders::_1),
        rcl_action_server_get_default_options(), server_options.callback_group);

    spdlog::info("Action server '{}' started", action_server_name);
}

void WaypointManagerNode::setup_action_client() {
    this->declare_parameter<std::string>("action_servers.reference_filter",
                                         "reference_filter");
    std::string reference_filter_server =
        this->get_parameter("action_servers.reference_filter").as_string();

    auto client_cb_group =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    reference_filter_client_ =
        rclcpp_action::create_client<ReferenceFilterAction>(
            this, reference_filter_server, client_cb_group);

    spdlog::info("Action client created for '{}'", reference_filter_server);
}

void WaypointManagerNode::setup_subscriber() {
    this->declare_parameter<std::string>("topics.pose", "pose");
    std::string pose_topic = this->get_parameter("topics.pose").as_string();

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos_sensor_data = rclcpp::QoS(
        rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    pose_sub_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        pose_topic, qos_sensor_data,
        std::bind(&WaypointManagerNode::pose_callback, this,
                  std::placeholders::_1),
        sub_options);

    spdlog::info("Subscribed to pose topic: {}", pose_topic);
}

rclcpp_action::GoalResponse WaypointManagerNode::handle_goal(
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const WaypointManagerAction::Goal> goal) {
    if (goal->waypoints.empty()) {
        spdlog::error("Received empty waypoint list");
        return rclcpp_action::GoalResponse::REJECT;
    }

    if (goal->target_server != "reference_filter") {
        spdlog::error(
            "Invalid target server: {}. Only 'reference_filter' is supported",
            goal->target_server);
        return rclcpp_action::GoalResponse::REJECT;
    }

    // If we have an active goal, we'll cancel it when we accept the new one
    if (nav_state_ != NavState::IDLE) {
        spdlog::info(
            "New goal received while already navigating. Previous goal will be "
            "aborted.");
    }

    spdlog::info("Accepting new goal with {} waypoints",
                 goal->waypoints.size());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse WaypointManagerNode::handle_cancel(
    const std::shared_ptr<GoalHandleWaypointManager> /*goal_handle*/) {
    spdlog::info("Received cancel request");

    // Cancel any ongoing reference filter action
    if (reference_filter_goal_handle_) {
        reference_filter_client_->async_cancel_goal(
            reference_filter_goal_handle_);
        reference_filter_goal_handle_.reset();
    }

    nav_state_ = NavState::IDLE;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void WaypointManagerNode::handle_accepted(
    const std::shared_ptr<GoalHandleWaypointManager> goal_handle) {
    // If we have an active goal, abort it
    if (current_goal_handle_ && current_goal_handle_->is_active()) {
        auto result = std::make_shared<WaypointManagerAction::Result>();
        result->success = false;
        result->completed_waypoints = completed_waypoints_;
        current_goal_handle_->abort(result);

        // Cancel any ongoing reference filter action
        if (reference_filter_goal_handle_) {
            reference_filter_client_->async_cancel_goal(
                reference_filter_goal_handle_);
            reference_filter_goal_handle_.reset();
        }
    }

    // Store the new goal handle
    current_goal_handle_ = goal_handle;

    // Get the goal
    auto goal = goal_handle->get_goal();

    // Store goal parameters
    waypoints_ = goal->waypoints;
    target_server_ = goal->target_server;
    current_waypoint_index_ = 0;
    completed_waypoints_ = 0;

    // Start navigation process
    start_navigation();
}

void WaypointManagerNode::start_navigation() {
    nav_state_ = NavState::NAVIGATING;
    send_next_waypoint();
}

void WaypointManagerNode::send_next_waypoint() {
    if (current_waypoint_index_ >= waypoints_.size()) {
        // All waypoints completed
        complete_navigation(true);
        return;
    }

    // Check if reference filter server is available
    if (!reference_filter_client_->wait_for_action_server(
            std::chrono::seconds(1))) {
        // Server not available - retry with exponential backoff
        auto retry_timer =
            this->create_wall_timer(std::chrono::seconds(1), [this]() {
                if (reference_filter_client_->wait_for_action_server(
                        std::chrono::milliseconds(100))) {
                    send_next_waypoint();
                }
            });

        spdlog::warn(
            "Reference filter action server not available, waiting...");
        return;
    }

    // Send current waypoint to reference filter
    auto goal_msg = ReferenceFilterAction::Goal();
    goal_msg.goal = waypoints_[current_waypoint_index_];

    spdlog::info("Sending waypoint {} to reference filter",
                 current_waypoint_index_);

    auto send_goal_options =
        rclcpp_action::Client<ReferenceFilterAction>::SendGoalOptions();

    // Use lambda functions instead of std::bind to avoid type mismatches
    send_goal_options.goal_response_callback =
        [this](const std::shared_ptr<GoalHandleReferenceFilter>& goal_handle) {
            if (!goal_handle) {
                spdlog::error("Reference filter rejected goal");
                complete_navigation(false);
                return;
            }

            reference_filter_goal_handle_ = goal_handle;
            spdlog::info("Reference filter accepted waypoint {}",
                         current_waypoint_index_);
        };

    send_goal_options.feedback_callback =
        [this](
            const std::shared_ptr<GoalHandleReferenceFilter>& /*goal_handle*/,
            const std::shared_ptr<const ReferenceFilterAction::Feedback>&
                feedback) {
            if (nav_state_ != NavState::WAITING_FOR_REFERENCE_FILTER ||
                !current_goal_handle_ || !current_goal_handle_->is_active()) {
                return;
            }

            // Forward feedback to our own clients
            auto wm_feedback =
                std::make_shared<WaypointManagerAction::Feedback>();

            // Convert reference filter feedback to waypoint manager feedback
            wm_feedback->current_pose.position.x = feedback->feedback.x;
            wm_feedback->current_pose.position.y = feedback->feedback.y;
            wm_feedback->current_pose.position.z = feedback->feedback.z;

            // Convert Euler angles to quaternion
            tf2::Quaternion q;
            q.setRPY(feedback->feedback.roll, feedback->feedback.pitch,
                     feedback->feedback.yaw);
            wm_feedback->current_pose.orientation = tf2::toMsg(q);

            wm_feedback->current_waypoint_index = current_waypoint_index_;

            // Calculate distance to current waypoint (informational only)
            if (current_waypoint_index_ < waypoints_.size()) {
                geometry_msgs::msg::Pose target_pose =
                    waypoints_[current_waypoint_index_].pose;
                wm_feedback->distance_to_waypoint =
                    calculate_distance(wm_feedback->current_pose, target_pose);
            } else {
                wm_feedback->distance_to_waypoint = 0.0;
            }

            current_goal_handle_->publish_feedback(wm_feedback);
        };

    send_goal_options.result_callback =
        [this](
            const typename GoalHandleReferenceFilter::WrappedResult& result) {
            if (nav_state_ != NavState::WAITING_FOR_REFERENCE_FILTER) {
                return;
            }

            reference_filter_goal_handle_.reset();

            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                spdlog::info("Waypoint {} reached successfully",
                             current_waypoint_index_);
                completed_waypoints_++;
                current_waypoint_index_++;
                nav_state_ = NavState::NAVIGATING;

                // Move to next waypoint
                send_next_waypoint();
            } else {
                // Handle failure or cancellation
                std::string status;
                switch (result.code) {
                    case rclcpp_action::ResultCode::ABORTED:
                        status = "ABORTED";
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        status = "CANCELED";
                        break;
                    default:
                        status = "UNKNOWN";
                        break;
                }

                spdlog::warn(
                    "Reference filter action failed for waypoint {}: {}",
                    current_waypoint_index_, status);

                complete_navigation(false);
            }
        };

    nav_state_ = NavState::WAITING_FOR_REFERENCE_FILTER;
    reference_filter_client_->async_send_goal(goal_msg, send_goal_options);
}

void WaypointManagerNode::complete_navigation(bool success) {
    if (!current_goal_handle_ || !current_goal_handle_->is_active()) {
        return;
    }

    auto result = std::make_shared<WaypointManagerAction::Result>();
    result->success = success;
    result->completed_waypoints = completed_waypoints_;

    if (success) {
        current_goal_handle_->succeed(result);
        spdlog::info("All waypoints completed successfully");
    } else {
        current_goal_handle_->abort(result);
        spdlog::error("Navigation failed");
    }

    nav_state_ = NavState::IDLE;
}

void WaypointManagerNode::pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    // Simple state update, no locks needed as this is atomic
    current_pose_ = msg->pose.pose;

    // Optionally publish feedback if we're actively navigating
    if (nav_state_ == NavState::WAITING_FOR_REFERENCE_FILTER &&
        current_goal_handle_ && current_goal_handle_->is_active()) {
        publish_feedback();
    }
}

void WaypointManagerNode::publish_feedback() {
    // Only called from pose_callback or feedback_callback
    if (!current_goal_handle_ || !current_goal_handle_->is_active() ||
        current_waypoint_index_ >= waypoints_.size()) {
        return;
    }

    auto feedback = std::make_shared<WaypointManagerAction::Feedback>();
    feedback->current_pose = current_pose_;
    feedback->current_waypoint_index = current_waypoint_index_;

    // Calculate distance to target (informational only)
    geometry_msgs::msg::Pose target_pose =
        waypoints_[current_waypoint_index_].pose;
    feedback->distance_to_waypoint =
        calculate_distance(current_pose_, target_pose);

    current_goal_handle_->publish_feedback(feedback);
}

double WaypointManagerNode::calculate_distance(
    const geometry_msgs::msg::Pose& pose1,
    const geometry_msgs::msg::Pose& pose2) {
    double dx = pose1.position.x - pose2.position.x;
    double dy = pose1.position.y - pose2.position.y;
    double dz = pose1.position.z - pose2.position.z;

    return std::sqrt(dx * dx + dy * dy + dz * dz);
}
