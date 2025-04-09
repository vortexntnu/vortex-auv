#include "waypoint_manager/waypoint_manager.hpp"
#include <functional>
#include <future>
#include <memory>
#include <thread>

using namespace std::placeholders;

WaypointManagerNode::WaypointManagerNode() : Node("waypoint_manager_node") {
    server_cb_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    client_cb_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    setup_action_server();
    setup_action_clients();
    setup_subscribers();

    // Start the worker thread
    running_ = true;
    worker_thread_ =
        std::thread(&WaypointManagerNode::process_waypoints_thread, this);

    spdlog::info("Waypoint Manager Node initialized");
}

WaypointManagerNode::~WaypointManagerNode() {
    // Signal thread to stop and wait for it to finish
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        running_ = false;
    }
    waypoint_cv_.notify_all();

    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }
}

void WaypointManagerNode::setup_action_server() {
    this->declare_parameter<std::string>("action_servers.waypoint_manager",
                                         "waypoint_manager");
    std::string action_server_name =
        this->get_parameter("action_servers.waypoint_manager").as_string();

    action_server_ = rclcpp_action::create_server<WaypointManagerAction>(
        this, action_server_name,
        std::bind(&WaypointManagerNode::handle_goal, this, _1, _2),
        std::bind(&WaypointManagerNode::handle_cancel, this, _1),
        std::bind(&WaypointManagerNode::handle_accepted, this, _1),
        rcl_action_server_get_default_options(), server_cb_group_);

    spdlog::info("Action server '{}' started", action_server_name);
}

void WaypointManagerNode::setup_action_clients() {
    this->declare_parameter<std::string>("action_servers.reference_filter",
                                         "reference_filter");
    std::string reference_filter_server =
        this->get_parameter("action_servers.reference_filter").as_string();

    reference_filter_client_ =
        rclcpp_action::create_client<ReferenceFilterAction>(
            this, reference_filter_server, client_cb_group_);

    spdlog::info("Action client created for '{}'", reference_filter_server);
}

void WaypointManagerNode::setup_subscribers() {
    this->declare_parameter<std::string>("topics.pose", "pose");
    std::string pose_topic = this->get_parameter("topics.pose").as_string();

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos_sensor_data = rclcpp::QoS(
        rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    pose_sub_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        pose_topic, qos_sensor_data,
        std::bind(&WaypointManagerNode::pose_callback, this, _1));

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

    // Check if there's an active goal
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (current_goal_handle_ && current_goal_handle_->is_active()) {
        spdlog::info("Preempting current goal for new goal");
        preempted_goal_id_ = current_goal_handle_->get_goal_id();
        // Signal the worker thread to check for preemption
        waypoint_cv_.notify_all();
    }

    spdlog::info("Accepting new goal with {} waypoints",
                 goal->waypoints.size());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse WaypointManagerNode::handle_cancel(
    const std::shared_ptr<GoalHandleWaypointManager> /*goal_handle*/) {
    spdlog::info("Received cancel request");

    // Signal the worker thread to check for cancellation
    waypoint_cv_.notify_all();

    return rclcpp_action::CancelResponse::ACCEPT;
}

void WaypointManagerNode::handle_accepted(
    const std::shared_ptr<GoalHandleWaypointManager> goal_handle) {
    std::lock_guard<std::mutex> lock(queue_mutex_);

    // Store the goal handle
    current_goal_handle_ = goal_handle;

    // Get the goal
    auto goal = goal_handle->get_goal();

    // Store goal parameters
    waypoints_ = goal->waypoints;
    target_server_ = goal->target_server;
    switching_threshold_ = goal->switching_threshold;
    current_waypoint_index_ = 0;
    completed_waypoints_ = 0;
    navigation_active_ = true;

    // Signal the worker thread that a new goal is available
    waypoint_cv_.notify_all();
}

void WaypointManagerNode::process_waypoints_thread() {
    auto feedback = std::make_shared<WaypointManagerAction::Feedback>();

    while (true) {
        std::shared_ptr<GoalHandleWaypointManager> goal_handle;
        std::vector<geometry_msgs::msg::PoseStamped> waypoints;
        size_t current_index;
        double threshold;
        bool should_process = false;

        // Wait for work or shutdown signal
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);

            waypoint_cv_.wait(lock, [this]() {
                return !running_ ||
                       (navigation_active_ &&
                        current_waypoint_index_ < waypoints_.size());
            });

            // Check if we're shutting down
            if (!running_) {
                break;
            }

            // Check if goal was preempted or canceled
            if (current_goal_handle_) {
                if (current_goal_handle_->get_goal_id() == preempted_goal_id_) {
                    spdlog::info("Goal was preempted");
                    auto result =
                        std::make_shared<WaypointManagerAction::Result>();
                    result->success = false;
                    result->completed_waypoints = completed_waypoints_;
                    current_goal_handle_->abort(result);
                    navigation_active_ = false;
                    continue;
                }

                if (current_goal_handle_->is_canceling()) {
                    spdlog::info("Goal was canceled");
                    auto result =
                        std::make_shared<WaypointManagerAction::Result>();
                    result->success = false;
                    result->completed_waypoints = completed_waypoints_;
                    current_goal_handle_->canceled(result);
                    navigation_active_ = false;
                    continue;
                }
            }

            // Check if we have an active goal and waypoints to process
            if (navigation_active_ &&
                current_waypoint_index_ < waypoints_.size()) {
                goal_handle = current_goal_handle_;
                waypoints = waypoints_;
                current_index = current_waypoint_index_;
                threshold = switching_threshold_;
                should_process = true;
            }
        }

        // Process the current waypoint (outside of lock)
        if (should_process && goal_handle) {
            geometry_msgs::msg::PoseStamped current_waypoint =
                waypoints[current_index];

            // Send waypoint to target server
            spdlog::info("Sending waypoint {} to target server", current_index);
            bool waypoint_executed =
                send_waypoint_to_target_server(current_waypoint);

            if (waypoint_executed) {
                // Check if we should move to the next waypoint based on
                // distance
                geometry_msgs::msg::Pose target_pose = current_waypoint.pose;
                geometry_msgs::msg::Pose current_pose;

                {
                    std::lock_guard<std::mutex> lock(queue_mutex_);
                    current_pose = current_pose_;
                }

                double distance = calculate_distance(current_pose, target_pose);

                // Update feedback
                {
                    std::lock_guard<std::mutex> lock(queue_mutex_);
                    feedback->current_pose = current_pose;
                    feedback->current_waypoint_index = current_index;
                    feedback->distance_to_waypoint = distance;
                    goal_handle->publish_feedback(feedback);
                }

                if (distance < threshold) {
                    spdlog::info("Reached waypoint {}, distance: {}",
                                 current_index, distance);

                    // Update waypoint index and completed count
                    {
                        std::lock_guard<std::mutex> lock(queue_mutex_);
                        current_waypoint_index_++;
                        completed_waypoints_++;

                        // Check if we've completed all waypoints
                        if (current_waypoint_index_ >= waypoints_.size()) {
                            spdlog::info("All waypoints completed");
                            auto result = std::make_shared<
                                WaypointManagerAction::Result>();
                            result->success = true;
                            result->completed_waypoints = completed_waypoints_;
                            goal_handle->succeed(result);
                            navigation_active_ = false;
                        }
                    }
                } else {
                    // Wait a bit before checking distance again
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            } else {
                // If waypoint execution failed, abort the goal
                std::lock_guard<std::mutex> lock(queue_mutex_);
                spdlog::error("Failed to execute waypoint {}", current_index);
                auto result = std::make_shared<WaypointManagerAction::Result>();
                result->success = false;
                result->completed_waypoints = completed_waypoints_;
                goal_handle->abort(result);
                navigation_active_ = false;
            }
        }
    }
}

void WaypointManagerNode::pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    current_pose_ = msg->pose.pose;
}

double WaypointManagerNode::calculate_distance(
    const geometry_msgs::msg::Pose& pose1,
    const geometry_msgs::msg::Pose& pose2) {
    double dx = pose1.position.x - pose2.position.x;
    double dy = pose1.position.y - pose2.position.y;
    double dz = pose1.position.z - pose2.position.z;

    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool WaypointManagerNode::send_waypoint_to_target_server(
    const geometry_msgs::msg::PoseStamped& waypoint) {
    std::promise<bool> waypoint_promise;
    std::future<bool> waypoint_future = waypoint_promise.get_future();

    if (!reference_filter_client_->wait_for_action_server(
            std::chrono::seconds(1))) {
        spdlog::error("Reference filter action server not available");
        return false;
    }

    auto goal_msg = ReferenceFilterAction::Goal();
    goal_msg.goal = waypoint;

    auto send_goal_options =
        rclcpp_action::Client<ReferenceFilterAction>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        [&waypoint_promise](const typename rclcpp_action::ClientGoalHandle<
                            ReferenceFilterAction>::SharedPtr& goal_handle) {
            if (!goal_handle) {
                spdlog::error("Reference filter goal was rejected");
                waypoint_promise.set_value(false);
            } else {
                spdlog::info("Reference filter goal accepted");
            }
        };

    send_goal_options.feedback_callback =
        [](const typename rclcpp_action::ClientGoalHandle<
               ReferenceFilterAction>::SharedPtr&,
           const std::shared_ptr<const ReferenceFilterAction::Feedback>) {
            // We're not using the feedback from the reference filter
        };

    send_goal_options.result_callback =
        [&waypoint_promise](const typename rclcpp_action::ClientGoalHandle<
                            ReferenceFilterAction>::WrappedResult& result) {
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    spdlog::info("Reference filter goal succeeded");
                    waypoint_promise.set_value(true);
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    spdlog::error("Reference filter goal was aborted");
                    waypoint_promise.set_value(false);
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    spdlog::info("Reference filter goal was canceled");
                    waypoint_promise.set_value(false);
                    break;
                default:
                    spdlog::error(
                        "Unknown result code from reference filter action");
                    waypoint_promise.set_value(false);
                    break;
            }
        };

    reference_filter_client_->async_send_goal(goal_msg, send_goal_options);

    // Wait for the waypoint execution to complete
    // This blocks until the reference filter action is complete
    try {
        return waypoint_future.get();
    } catch (const std::exception& e) {
        spdlog::error("Exception while waiting for waypoint execution: {}",
                      e.what());
        return false;
    }
}
