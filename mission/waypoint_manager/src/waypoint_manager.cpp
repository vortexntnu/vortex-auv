#include "waypoint_manager/waypoint_manager.hpp"
#include <functional>
#include <future>
#include <memory>
#include <thread>

using namespace std::placeholders;

WaypointManagerNode::WaypointManagerNode() : Node("waypoint_manager_node") {
    // Create callback groups for server and client
    server_cb_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    client_cb_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // Setup action server, clients and subscribers
    setup_action_server();
    setup_action_clients();
    setup_subscribers();

    RCLCPP_INFO(this->get_logger(), "Waypoint Manager Node initialized");
}

void WaypointManagerNode::setup_action_server() {
    // Declare parameters for action server name
    this->declare_parameter<std::string>("action_servers.waypoint_manager",
                                         "waypoint_manager");
    std::string action_server_name =
        this->get_parameter("action_servers.waypoint_manager").as_string();

    // Create the action server
    action_server_ = rclcpp_action::create_server<WaypointManagerAction>(
        this, action_server_name,
        std::bind(&WaypointManagerNode::handle_goal, this, _1, _2),
        std::bind(&WaypointManagerNode::handle_cancel, this, _1),
        std::bind(&WaypointManagerNode::handle_accepted, this, _1),
        rcl_action_server_get_default_options(), server_cb_group_);

    RCLCPP_INFO(this->get_logger(), "Action server '%s' started",
                action_server_name.c_str());
}

void WaypointManagerNode::setup_action_clients() {
    // Declare parameters for client action server names
    this->declare_parameter<std::string>("action_servers.reference_filter",
                                         "reference_filter");
    std::string reference_filter_server =
        this->get_parameter("action_servers.reference_filter").as_string();

    // Create action clients
    reference_filter_client_ =
        rclcpp_action::create_client<ReferenceFilterAction>(
            this, reference_filter_server, client_cb_group_);

    RCLCPP_INFO(this->get_logger(), "Action client created for '%s'",
                reference_filter_server.c_str());
}

void WaypointManagerNode::setup_subscribers() {
    // Declare parameters for subscription topics
    this->declare_parameter<std::string>("topics.pose", "pose");
    std::string pose_topic = this->get_parameter("topics.pose").as_string();

    // Quality of service profile
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos_sensor_data = rclcpp::QoS(
        rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    // Create pose subscription
    pose_sub_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        pose_topic, qos_sensor_data,
        std::bind(&WaypointManagerNode::pose_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Subscribed to pose topic: %s",
                pose_topic.c_str());
}

rclcpp_action::GoalResponse WaypointManagerNode::handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const WaypointManagerAction::Goal> goal) {
    (void)uuid;  // Unused

    // Check if goal is valid
    if (goal->waypoints.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Received empty waypoint list");
        return rclcpp_action::GoalResponse::REJECT;
    }

    // Check if target server is valid - only allow reference_filter
    if (goal->target_server != "reference_filter") {
        RCLCPP_ERROR(
            this->get_logger(),
            "Invalid target server: %s. Only 'reference_filter' is supported",
            goal->target_server.c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }

    // If there's an active goal, preempt it
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (current_goal_handle_ && current_goal_handle_->is_active()) {
            RCLCPP_INFO(this->get_logger(),
                        "Preempting current goal for new goal");
            preempted_goal_id_ = current_goal_handle_->get_goal_id();
        }
    }

    RCLCPP_INFO(this->get_logger(), "Accepting new goal with %zu waypoints",
                goal->waypoints.size());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse WaypointManagerNode::handle_cancel(
    const std::shared_ptr<GoalHandleWaypointManager> goal_handle) {
    (void)goal_handle;  // Unused
    RCLCPP_INFO(this->get_logger(), "Received cancel request");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void WaypointManagerNode::handle_accepted(
    const std::shared_ptr<GoalHandleWaypointManager> goal_handle) {
    // Start a new thread to process the goal
    std::thread{
        std::bind(&WaypointManagerNode::execute_waypoint_navigation, this, _1),
        goal_handle}
        .detach();
}

void WaypointManagerNode::execute_waypoint_navigation(
    const std::shared_ptr<GoalHandleWaypointManager> goal_handle) {
    // Set current goal handle and extract goal information
    {
        std::lock_guard<std::mutex> lock(mutex_);
        current_goal_handle_ = goal_handle;
        navigation_active_ = true;

        auto goal = goal_handle->get_goal();
        waypoints_ = goal->waypoints;
        target_server_ = goal->target_server;
        switching_threshold_ = goal->switching_threshold;
        current_waypoint_index_ = 0;
    }

    RCLCPP_INFO(this->get_logger(),
                "Starting waypoint navigation with %zu waypoints",
                waypoints_.size());

    // Create result and feedback objects
    auto result = std::make_shared<WaypointManagerAction::Result>();
    auto feedback = std::make_shared<WaypointManagerAction::Feedback>();

    // Main execution loop
    while (rclcpp::ok()) {
        // Check if goal was canceled or preempted
        {
            std::lock_guard<std::mutex> lock(mutex_);

            if (goal_handle->get_goal_id() == preempted_goal_id_) {
                RCLCPP_INFO(this->get_logger(), "Goal was preempted");
                result->success = false;
                result->completed_waypoints = current_waypoint_index_;
                goal_handle->abort(result);
                navigation_active_ = false;
                return;
            }

            if (goal_handle->is_canceling()) {
                RCLCPP_INFO(this->get_logger(), "Goal was canceled");
                result->success = false;
                result->completed_waypoints = current_waypoint_index_;
                goal_handle->canceled(result);
                navigation_active_ = false;
                return;
            }
        }

        // Check if we've completed all waypoints
        if (current_waypoint_index_ >= waypoints_.size()) {
            RCLCPP_INFO(this->get_logger(), "All waypoints completed");
            result->success = true;
            result->completed_waypoints = waypoints_.size();
            goal_handle->succeed(result);

            std::lock_guard<std::mutex> lock(mutex_);
            navigation_active_ = false;
            return;
        }

        // Send current waypoint to target server (if not already navigating)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!current_waypoint_promise_) {
                RCLCPP_INFO(this->get_logger(),
                            "Sending waypoint %zu to target server",
                            current_waypoint_index_);
                send_waypoint_to_target_server(
                    waypoints_[current_waypoint_index_]);
            }
        }

        // Calculate distance to current waypoint and check if we've reached it
        geometry_msgs::msg::Pose target_pose =
            waypoints_[current_waypoint_index_].pose;
        geometry_msgs::msg::Pose current_pose;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            current_pose = current_pose_;
        }

        double distance = calculate_distance(current_pose, target_pose);

        // Update feedback
        {
            std::lock_guard<std::mutex> lock(mutex_);
            feedback->current_pose = current_pose;
            feedback->current_waypoint_index = current_waypoint_index_;
            feedback->distance_to_waypoint = distance;
            goal_handle->publish_feedback(feedback);
        }

        // Check if we've reached the waypoint
        if (distance < switching_threshold_) {
            RCLCPP_INFO(this->get_logger(),
                        "Reached waypoint %zu, distance: %f",
                        current_waypoint_index_, distance);

            // If we have a promise, complete it (to cancel any pending target
            // server action)
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (current_waypoint_promise_) {
                    current_waypoint_promise_->set_value(true);
                    current_waypoint_promise_.reset();
                }
            }

            // Move to next waypoint
            current_waypoint_index_++;
        }

        // Sleep to avoid busy waiting
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void WaypointManagerNode::pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    current_pose_ = msg->pose.pose;
}

double WaypointManagerNode::calculate_distance(
    const geometry_msgs::msg::Pose& pose1,
    const geometry_msgs::msg::Pose& pose2) {
    // Calculate Euclidean distance between two poses (position only)
    double dx = pose1.position.x - pose2.position.x;
    double dy = pose1.position.y - pose2.position.y;
    double dz = pose1.position.z - pose2.position.z;

    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void WaypointManagerNode::send_waypoint_to_target_server(
    const geometry_msgs::msg::PoseStamped& waypoint) {
    // Create a new promise for this waypoint
    current_waypoint_promise_ = std::make_shared<std::promise<bool>>();

    // Check if reference filter client is ready
    if (!reference_filter_client_->wait_for_action_server(
            std::chrono::seconds(1))) {
        RCLCPP_ERROR(this->get_logger(),
                     "Reference filter action server not available");
        current_waypoint_promise_->set_value(false);
        current_waypoint_promise_.reset();
        return;
    }

    // Create goal
    auto goal_msg = ReferenceFilterAction::Goal();
    goal_msg.goal = waypoint;

    // Send goal with inline lambdas that have the correct signatures
    auto send_goal_options =
        rclcpp_action::Client<ReferenceFilterAction>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        [this](const rclcpp_action::ClientGoalHandle<
               ReferenceFilterAction>::SharedPtr& goal_handle) {
            if (!goal_handle) {
                RCLCPP_ERROR(this->get_logger(),
                             "Reference filter goal was rejected");
                std::lock_guard<std::mutex> lock(mutex_);
                if (current_waypoint_promise_) {
                    current_waypoint_promise_->set_value(false);
                    current_waypoint_promise_.reset();
                }
            } else {
                RCLCPP_INFO(this->get_logger(),
                            "Reference filter goal accepted");
            }
        };

    send_goal_options.feedback_callback =
        [this](rclcpp_action::ClientGoalHandle<ReferenceFilterAction>::SharedPtr
                   goal_handle,
               const std::shared_ptr<const ReferenceFilterAction::Feedback>
                   feedback) {
            (void)goal_handle;  // Unused
            (void)feedback;     // Unused, we're not using the feedback from the
                                // reference filter
        };

    send_goal_options.result_callback =
        [this](const rclcpp_action::ClientGoalHandle<
               ReferenceFilterAction>::WrappedResult& result) {
            std::lock_guard<std::mutex> lock(mutex_);

            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(),
                                "Reference filter goal succeeded");
                    if (current_waypoint_promise_) {
                        current_waypoint_promise_->set_value(true);
                        current_waypoint_promise_.reset();
                    }
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(),
                                 "Reference filter goal was aborted");
                    if (current_waypoint_promise_) {
                        current_waypoint_promise_->set_value(false);
                        current_waypoint_promise_.reset();
                    }
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_INFO(this->get_logger(),
                                "Reference filter goal was canceled");
                    if (current_waypoint_promise_) {
                        current_waypoint_promise_->set_value(false);
                        current_waypoint_promise_.reset();
                    }
                    break;
                default:
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "Unknown result code from reference filter action");
                    if (current_waypoint_promise_) {
                        current_waypoint_promise_->set_value(false);
                        current_waypoint_promise_.reset();
                    }
                    break;
            }
        };

    reference_filter_client_->async_send_goal(goal_msg, send_goal_options);
}
