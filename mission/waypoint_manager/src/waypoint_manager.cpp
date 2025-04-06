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

    spdlog::info("Waypoint Manager Node initialized");
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
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const WaypointManagerAction::Goal> goal) {
    (void)uuid;  // Unused

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

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (current_goal_handle_ && current_goal_handle_->is_active()) {
            spdlog::info("Preempting current goal for new goal");
            preempted_goal_id_ = current_goal_handle_->get_goal_id();
        }
    }

    spdlog::info("Accepting new goal with {} waypoints",
                 goal->waypoints.size());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse WaypointManagerNode::handle_cancel(
    const std::shared_ptr<GoalHandleWaypointManager> goal_handle) {
    (void)goal_handle;  // Unused
    spdlog::info("Received cancel request");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void WaypointManagerNode::handle_accepted(
    const std::shared_ptr<GoalHandleWaypointManager> goal_handle) {
    std::thread{
        std::bind(&WaypointManagerNode::execute_waypoint_navigation, this, _1),
        goal_handle}
        .detach();
}

void WaypointManagerNode::execute_waypoint_navigation(
    const std::shared_ptr<GoalHandleWaypointManager> goal_handle) {
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

    spdlog::info("Starting waypoint navigation with {} waypoints",
                 waypoints_.size());

    auto result = std::make_shared<WaypointManagerAction::Result>();
    auto feedback = std::make_shared<WaypointManagerAction::Feedback>();

    while (rclcpp::ok()) {
        {
            std::lock_guard<std::mutex> lock(mutex_);

            if (goal_handle->get_goal_id() == preempted_goal_id_) {
                spdlog::info("Goal was preempted");
                result->success = false;
                result->completed_waypoints = current_waypoint_index_;
                goal_handle->abort(result);
                navigation_active_ = false;
                return;
            }

            if (goal_handle->is_canceling()) {
                spdlog::info("Goal was canceled");
                result->success = false;
                result->completed_waypoints = current_waypoint_index_;
                goal_handle->canceled(result);
                navigation_active_ = false;
                return;
            }
        }

        if (current_waypoint_index_ >= waypoints_.size()) {
            spdlog::info("All waypoints completed");
            result->success = true;
            result->completed_waypoints = waypoints_.size();
            goal_handle->succeed(result);

            std::lock_guard<std::mutex> lock(mutex_);
            navigation_active_ = false;
            return;
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!current_waypoint_promise_) {
                spdlog::info("Sending waypoint {} to target server",
                             current_waypoint_index_);
                send_waypoint_to_target_server(
                    waypoints_[current_waypoint_index_]);
            }
        }

        geometry_msgs::msg::Pose target_pose =
            waypoints_[current_waypoint_index_].pose;
        geometry_msgs::msg::Pose current_pose;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            current_pose = current_pose_;
        }

        double distance = calculate_distance(current_pose, target_pose);

        {
            std::lock_guard<std::mutex> lock(mutex_);
            feedback->current_pose = current_pose;
            feedback->current_waypoint_index = current_waypoint_index_;
            feedback->distance_to_waypoint = distance;
            goal_handle->publish_feedback(feedback);
        }

        if (distance < switching_threshold_) {
            spdlog::info("Reached waypoint {}, distance: {}",
                         current_waypoint_index_, distance);

            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (current_waypoint_promise_) {
                    current_waypoint_promise_->set_value(true);
                    current_waypoint_promise_.reset();
                }
            }

            current_waypoint_index_++;
        }

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
    double dx = pose1.position.x - pose2.position.x;
    double dy = pose1.position.y - pose2.position.y;
    double dz = pose1.position.z - pose2.position.z;

    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void WaypointManagerNode::send_waypoint_to_target_server(
    const geometry_msgs::msg::PoseStamped& waypoint) {
    current_waypoint_promise_ = std::make_shared<std::promise<bool>>();

    if (!reference_filter_client_->wait_for_action_server(
            std::chrono::seconds(1))) {
        spdlog::error("Reference filter action server not available");
        current_waypoint_promise_->set_value(false);
        current_waypoint_promise_.reset();
        return;
    }

    auto goal_msg = ReferenceFilterAction::Goal();
    goal_msg.goal = waypoint;

    auto send_goal_options =
        rclcpp_action::Client<ReferenceFilterAction>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        [this](const rclcpp_action::ClientGoalHandle<
               ReferenceFilterAction>::SharedPtr& goal_handle) {
            if (!goal_handle) {
                spdlog::error("Reference filter goal was rejected");
                std::lock_guard<std::mutex> lock(mutex_);
                if (current_waypoint_promise_) {
                    current_waypoint_promise_->set_value(false);
                    current_waypoint_promise_.reset();
                }
            } else {
                spdlog::info("Reference filter goal accepted");
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
                    spdlog::info("Reference filter goal succeeded");
                    if (current_waypoint_promise_) {
                        current_waypoint_promise_->set_value(true);
                        current_waypoint_promise_.reset();
                    }
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    spdlog::error("Reference filter goal was aborted");
                    if (current_waypoint_promise_) {
                        current_waypoint_promise_->set_value(false);
                        current_waypoint_promise_.reset();
                    }
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    spdlog::info("Reference filter goal was canceled");
                    if (current_waypoint_promise_) {
                        current_waypoint_promise_->set_value(false);
                        current_waypoint_promise_.reset();
                    }
                    break;
                default:
                    spdlog::error(
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
