#include "roll_guidance/roll_guidance_ros.hpp"

#include <chrono>
#include <cmath>
#include <future>
#include <thread>
#include <vector>
#include <stdexcept>

#include <spdlog/spdlog.h>
#include <vortex/utils/ros/qos_profiles.hpp>
#include <vortex_msgs/msg/waypoint.hpp>
#include <vortex_msgs/msg/waypoint_mode.hpp>

namespace vortex::guidance {

/**
 * @brief Constructs the ROLL guidance ROS 2 node.
 *
 * Initialises subscribers, the ROLL action server, the roll
 * manoeuver generator, and the Waypoint Manager action client.
 *
 * @param options ROS 2 node configuration options.
 */
ROLLGuidanceNode::ROLLGuidanceNode(
    const rclcpp::NodeOptions& options)
    : Node("roll_guidance_node", options) {

    set_subscribers();
    set_action_server();
    set_roll_guidance();
    set_waypoint_manager_client();

    /**
     * @brief Disables global ROS shutdown during integration tests.
     *
     * Defaults to false so normal AUV operation is unchanged.
     */
    this->declare_parameter<bool>("test.disable_shutdown", false);

    /**
     * @brief Creates a watchdog timer to monitor pose and gate freshness.
     */
    watchdog_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(
            &ROLLGuidanceNode::watchdog_callback,
            this));

    spdlog::info("ROLL guidance node started");
}

/**
 * @brief Initialises subscribers for the AUV pose and gate position.
 *
 * Subscribes to the configured pose and waypoint topics.
 * Incoming messages update the current AUV pose and target gate.
 */
void ROLLGuidanceNode::set_subscribers() {
    this->declare_parameter<std::string>("topics.pose");
    this->declare_parameter<std::string>("topics.waypoint");

    const std::string pose_topic =
        this->get_parameter("topics.pose").as_string();

    const std::string waypoint_topic =
        this->get_parameter("topics.waypoint").as_string();

    auto qos = vortex::utils::qos_profiles::sensor_data_profile(1);

    pose_sub_ =
        this->create_subscription<
            geometry_msgs::msg::PoseWithCovarianceStamped>(
            pose_topic,
            qos,
            std::bind(
                &ROLLGuidanceNode::pose_callback,
                this,
                std::placeholders::_1));

    waypoint_sub_ =
        this->create_subscription<geometry_msgs::msg::PointStamped>(
            waypoint_topic,
            rclcpp::QoS(10),
            std::bind(
                &ROLLGuidanceNode::waypoint_callback,
                this,
                std::placeholders::_1));

    const std::string killswitch_topic =
        this->declare_parameter<std::string>(
            "topics.killswitch", "killswitch");

    killswitch_sub_ =
        this->create_subscription<std_msgs::msg::Bool>(
            killswitch_topic,
            rclcpp::QoS(10).reliable(),
            std::bind(
                &ROLLGuidanceNode::killswitch_callback,
                this,
                std::placeholders::_1));
}

/**
 * @brief Creates the ROLL guidance action server.
 *
 * Registers callbacks for accepting, canceling, and executing
 * ROLL guidance goals.
 */
void ROLLGuidanceNode::set_action_server() {
    this->declare_parameter<std::string>("action_servers.roll");

    const std::string action_server_name =
        this->get_parameter("action_servers.roll").as_string();

    cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);

    action_server_ =
        rclcpp_action::create_server<GuidanceWaypoint>(
            this,
            action_server_name,
            std::bind(
                &ROLLGuidanceNode::handle_goal,
                this,
                std::placeholders::_1,
                std::placeholders::_2),
            std::bind(
                &ROLLGuidanceNode::handle_cancel,
                this,
                std::placeholders::_1),
            std::bind(
                &ROLLGuidanceNode::handle_accepted,
                this,
                std::placeholders::_1),
            rcl_action_server_get_default_options(),
            cb_group_);
}

/**
 * @brief Initialises the Waypoint Manager action client.
 *
 * Creates a client used to send generated roll waypoints
 * to the Waypoint Manager.
 */
void ROLLGuidanceNode::set_waypoint_manager_client() {
    this->declare_parameter<std::string>(
        "action_servers.waypoint_manager",
        "waypoint_manager");

    const std::string action_name =
        this->get_parameter(
            "action_servers.waypoint_manager").as_string();

    waypoint_manager_client_ =
        rclcpp_action::create_client<WaypointManager>(
            this,
            action_name);
}

/**
 * @brief Configures the roll manoeuver generator and gate plane.
 *
 * Reads the roll angle, number of steps, trigger distance,
 * and gate-plane normal from ROS 2 parameters.
 *
 * @throws std::invalid_argument If the gate normal is invalid
 *         or the roll generator parameters are invalid.
 */
void ROLLGuidanceNode::set_roll_guidance() {
    this->declare_parameter<double>(
        "roll.roll_angle", 2.0 * M_PI);

    this->declare_parameter<int>(
        "roll.number_of_steps", 8);

    this->declare_parameter<double>(
        "roll.trigger_distance", 3.0);

    params_.roll_angle =
        this->get_parameter("roll.roll_angle").as_double();

    params_.number_of_steps =
        this->get_parameter("roll.number_of_steps").as_int();

    params_.trigger_distance =
        this->get_parameter("roll.trigger_distance").as_double();

    roll_guidance_ =
        std::make_unique<RollManeuverGenerator>(params_);

    const auto normal =
        this->declare_parameter<std::vector<double>>(
            "roll.gate_normal",
            {1.0, 0.0, 0.0});

    if (normal.size() != 3) {
        throw std::invalid_argument(
            "roll.gate_normal must contain exactly 3 values");
    }

    gate_normal_ = Eigen::Vector3d(
        normal[0],
        normal[1],
        normal[2]);

    if (!gate_normal_.allFinite() ||
        gate_normal_.norm() < 1e-9) {
        throw std::invalid_argument(
            "roll.gate_normal must be finite and nonzero");
    }

    gate_normal_.normalize();

    roll_tolerance_ = this->declare_parameter<double>(
        "roll.roll_tolerance_degrees", 5.0)
        * M_PI / 180.0;

    orientation_tolerance_ = this->declare_parameter<double>(
        "roll.orientation_tolerance_degrees", 5.0)
        * M_PI / 180.0;

    pose_timeout_seconds_ =
    this->declare_parameter<double>(
        "roll.pose_timeout_seconds", 0.5);

    trigger_timeout_seconds_ =
        this->declare_parameter<double>(
            "roll.trigger_timeout_seconds", 30.0);

    maneuver_timeout_seconds_ =
        this->declare_parameter<double>(
            "roll.maneuver_timeout_seconds", 120.0);

    if (!std::isfinite(roll_tolerance_) ||
        !std::isfinite(orientation_tolerance_) ||
        !std::isfinite(pose_timeout_seconds_) ||
        !std::isfinite(trigger_timeout_seconds_) ||
        !std::isfinite(maneuver_timeout_seconds_) ||
        roll_tolerance_ <= 0.0 ||
        orientation_tolerance_ <= 0.0 ||
        pose_timeout_seconds_ <= 0.0 ||
        trigger_timeout_seconds_ <= 0.0 ||
        maneuver_timeout_seconds_ <= 0.0) {

        throw std::invalid_argument(
            "ROLL tolerances and timeouts must be finite and positive");
    }
}

/**
 * @brief Determines whether the AUV is close enough to start rolling.
 *
 * Calculates the Euclidean distance between the AUV and gate.
 *
 * @param current_position Current AUV position in meters.
 * @param gate_position Gate position in meters.
 * @return true if the distance is within the trigger distance.
 */
bool ROLLGuidanceNode::should_start_roll(
    const Eigen::Vector3d& current_position,
    const Eigen::Vector3d& gate_position) const {

    return (gate_position - current_position).norm()
        <= params_.trigger_distance;
}

/**
 * @brief Processes a measured AUV pose.
 *
 * Validates the measurement, updates the last valid pose time,
 * tracks accumulated roll, and detects gate crossing.
 *
 * @param msg Incoming pose measurement.
 */
void ROLLGuidanceNode::pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {

    const auto& p = msg->pose.pose.position;
    const auto& o = msg->pose.pose.orientation;

    const Eigen::Vector3d position(p.x, p.y, p.z);
    const Eigen::Quaterniond orientation(
        o.w, o.x, o.y, o.z);

    if (!position.allFinite() ||
        !orientation.coeffs().allFinite() ||
        !std::isfinite(orientation.norm()) ||
        orientation.norm() < 1e-9) {

        RCLCPP_WARN(
            get_logger(),
            "Ignoring invalid AUV pose");

        return;
    }

    vortex::utils::types::Pose pose;
    pose.set_pos(position);
    pose.set_ori(orientation.normalized());

    std::shared_ptr<GoalHandleROLL> goal_to_finish;

    {
        std::lock_guard<std::mutex> lock(mutex_);

        current_pose_ = pose;
        pose_received_ = true;
        last_pose_time_ = SteadyClock::now();

        if (roll_tracking_active_ &&
            !roll_tracking_error_ &&
            !finishing_.load()) {

            try {
                roll_tracker_.observe(
                    pose.ori_quaternion());
            } catch (const std::exception& e) {
                roll_tracking_error_ = true;

                RCLCPP_ERROR(
                    get_logger(),
                    "Roll tracking failed: %s",
                    e.what());
            }
        }

        if (gate_received_ && !finishing_.load()) {
            if (has_passed_gate(position)) {
                goal_to_finish = goal_handle_;
            }
        }
    }

    if (goal_to_finish) {
        finish_roll(
            goal_to_finish,
            FinishReason::GatePassed);
    }
}

/**
 * @brief Processes an incoming gate waypoint.
 *
 * Updates the gate position and resets crossing detection
 * before an active manoeuver begins.
 *
 * @param msg Incoming gate position message.
 *
 * @note Gate and pose positions must use the same coordinate frame.
 */
void ROLLGuidanceNode::waypoint_callback(
    const geometry_msgs::msg::PointStamped::SharedPtr msg) {

    std::lock_guard<std::mutex> lock(mutex_);

    if (goal_handle_ || finishing_.load()) {
        return;
    }

    gate_position_ = Eigen::Vector3d(
        msg->point.x,
        msg->point.y,
        msg->point.z);

    gate_received_ = true;
    gate_approach_seen_ = false;
    gate_passed_ = false;

    spdlog::info("Received ROLL gate waypoint");
}

/**
 * @brief Aborts the ROLL action when the killswitch is engaged.
 *
 * The DP controller is responsible for stopping wrench publication.
 * This callback only stops the guidance action and requests
 * cancellation of the downstream Waypoint Manager goal.
 *
 * @param msg Current killswitch state.
 */
void ROLLGuidanceNode::killswitch_callback(
    const std_msgs::msg::Bool::SharedPtr msg) {

    std::shared_ptr<GoalHandleROLL> goal_to_abort;

    {
        std::lock_guard<std::mutex> lock(mutex_);

        killswitch_received_ = true;
        killswitch_on_ = msg->data;

        if (killswitch_on_ && !finishing_.load()) {
            goal_to_abort = goal_handle_;
        }
    }

    if (goal_to_abort) {
        RCLCPP_WARN(
            get_logger(),
            "Killswitch engaged: aborting ROLL guidance");

        finish_roll(
            goal_to_abort,
            FinishReason::Failed);
    }
}

/**
 * @brief Monitors the active ROLL action for cancellation,
 * killswitch activation, stale pose, and timeouts.
 *
 * Uses a steady clock for elapsed-time measurements.
 * Finalises the action outside mutex_.
 */
void ROLLGuidanceNode::watchdog_callback() {

    std::shared_ptr<GoalHandleROLL> goal;
    FinishReason reason = FinishReason::Failed;
    bool should_finish = false;
    const auto now = SteadyClock::now();

    {
        std::lock_guard<std::mutex> lock(mutex_);

        goal = goal_handle_;

        if (!goal || finishing_.load()) {
            return;
        }

        const double goal_elapsed =
            std::chrono::duration<double>(
                now - goal_start_time_).count();

        const double pose_age =
            pose_received_
                ? std::chrono::duration<double>(
                      now - last_pose_time_).count()
                : goal_elapsed;

        if (cancel_requested_ || goal->is_canceling()) {
            reason = FinishReason::Canceled;
            should_finish = true;

        } else if (killswitch_received_ && killswitch_on_) {
            RCLCPP_ERROR(
                get_logger(),
                "ROLL aborted: killswitch engaged");

            should_finish = true;

        } else if (pose_age > pose_timeout_seconds_) {
            RCLCPP_ERROR(
                get_logger(),
                "ROLL aborted: pose timeout (%.2f s)",
                pose_age);

            should_finish = true;

        } else if (roll_tracking_error_) {
            RCLCPP_ERROR(
                get_logger(),
                "ROLL aborted: invalid roll tracking");

            should_finish = true;

        } else if (!maneuver_started_ &&
                   goal_elapsed > trigger_timeout_seconds_) {

            RCLCPP_ERROR(
                get_logger(),
                "ROLL aborted: trigger timeout");

            should_finish = true;

        } else if (maneuver_started_) {
            const double maneuver_elapsed =
                std::chrono::duration<double>(
                    now - maneuver_start_time_).count();

            if (maneuver_elapsed >
                maneuver_timeout_seconds_) {

                RCLCPP_ERROR(
                    get_logger(),
                    "ROLL aborted: maneuver timeout");

                should_finish = true;
            }
        }
    }

    if (should_finish) {
        finish_roll(goal, reason);
    }
}

/**
 * @brief Evaluates an incoming ROLL guidance action goal.
 *
 * Accepts the first goal and rejects subsequent goals.
 * This node is designed to execute one ROLL manoeuver before
 * shutting down.
 *
 * @param uuid Unique goal identifier.
 * @param goal Requested guidance goal.
 * @return ACCEPT_AND_EXECUTE for the first goal, REJECT otherwise.
 */
rclcpp_action::GoalResponse ROLLGuidanceNode::handle_goal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const GuidanceWaypoint::Goal> goal) {

    (void)goal;

    if (finishing_.load() ||
        shutdown_requested_.load() ||
        goal_claimed_.exchange(true)) {

        spdlog::warn(
            "ROLL goal rejected: already running or shutting down");

        return rclcpp_action::GoalResponse::REJECT;
    }

    spdlog::info("Accepted ROLL goal");

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

/**
 * @brief Accepts cancellation of a ROLL action.
 *
 * Marks cancellation as requested. The watchdog then
 * finalises the ROLL action and requests cancellation
 * of the downstream Waypoint Manager goal.
 *
 * @param goal_handle ROLL goal being canceled.
 * @return ACCEPT.
 */
rclcpp_action::CancelResponse ROLLGuidanceNode::handle_cancel(
    const std::shared_ptr<GoalHandleROLL> goal_handle) {

    (void)goal_handle;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        cancel_requested_ = true;
    }

    RCLCPP_INFO(
        get_logger(),
        "ROLL cancellation requested");

    return rclcpp_action::CancelResponse::ACCEPT;
}

/**
 * @brief Starts execution of an accepted ROLL goal.
 *
 * Launches a separate thread to execute the manoeuver without
 * blocking the ROS 2 action server callback.
 *
 * @param goal_handle Handle of the accepted goal.
 *
 * @warning The node must remain alive until execution finishes.
 */
void ROLLGuidanceNode::handle_accepted(
    const std::shared_ptr<GoalHandleROLL> goal_handle) {

    std::thread(
        [this, goal_handle]() {
            execute(goal_handle);
        }
    ).detach();
}

/**
 * @brief Detects whether the AUV has crossed the gate plane.
 *
 * Calculates signed distance from the gate plane using the
 * configured gate normal.
 *
 * The AUV must first be observed on the approach side,
 * then move to the exit side.
 *
 * @param position Current AUV position.
 * @return true if the AUV has crossed the gate plane.
 *
 * @note Must be called while holding mutex_.
 */
bool ROLLGuidanceNode::has_passed_gate(
    const Eigen::Vector3d& position) {

    constexpr double tolerance = 0.05;

    const double signed_distance =
        (position - gate_position_).dot(gate_normal_);

    if (signed_distance < -tolerance) {
        gate_approach_seen_ = true;
    }

    if (gate_approach_seen_ &&
        signed_distance > tolerance) {

        gate_passed_ = true;
    }

    return gate_passed_;
}

/**
 * @brief Verifies that the requested roll was physically completed.
 *
 * Checks both signed accumulated roll and the final quaternion.
 *
 * @return True if both checks pass.
 * @note Caller must hold mutex_.
 */
bool ROLLGuidanceNode::roll_verified() const {

    if (!roll_tracking_active_ ||
        roll_tracking_error_ ||
        !pose_received_ ||
        !roll_tracker_.initialized()) {
        return false;
    }

    const auto now = SteadyClock::now();

    const double pose_age =
        std::chrono::duration<double>(
            now - last_pose_time_).count();

    if (pose_age > pose_timeout_seconds_) {
        return false;
    }

    const bool accumulated_roll_ok =
        roll_tracker_.completed(
            params_.roll_angle,
            roll_tolerance_);

    const Eigen::Quaterniond desired_final =
        roll_initial_orientation_ *
        Eigen::Quaterniond(
            Eigen::AngleAxisd(
                params_.roll_angle,
                Eigen::Vector3d::UnitX()));

    const Eigen::Quaterniond measured =
        current_pose_.ori_quaternion().normalized();

    const bool final_orientation_ok =
        measured.angularDistance(
            desired_final.normalized()) <=
        orientation_tolerance_;

    return accumulated_roll_ok && final_orientation_ok;
}

/**
 * @brief Finalises the ROLL action exactly once.
 *
 * Prevents multiple callbacks from completing the same action.
 *
 * If the gate is passed before the roll finishes, requests
 * cancellation of the active Waypoint Manager goal.
 *
 * Completes the ROLL action and initiates node shutdown.
 *
 * @param goal_handle Active ROLL action goal.
 * @param reason Reason why the action is ending.
 *
 * @note Must be called without holding mutex_.
 */
void ROLLGuidanceNode::finish_roll(
    const std::shared_ptr<GoalHandleROLL>& goal_handle,
    FinishReason reason) {

    if (finishing_.exchange(true)) {
        return;
    }

    WaypointManagerGoalHandle::SharedPtr manager_handle;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        manager_handle = waypoint_manager_goal_handle_;
    }

    if (reason != FinishReason::Completed &&
        manager_handle) {

        waypoint_manager_client_->async_cancel_goal(
            manager_handle);
    }

    auto result = std::make_shared<GuidanceWaypoint::Result>();

    result->success = (reason == FinishReason::Completed);

    if (goal_handle && goal_handle->is_active()) {
        if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
        } else if (result->success) {
            goal_handle->succeed(result);
        } else {
            goal_handle->abort(result);
        }
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (goal_handle_ == goal_handle) {
            goal_handle_.reset();
        }

        waypoint_manager_goal_handle_.reset();
    }

}

/**
 * @brief Schedules a one-time ROS 2 shutdown.
 *
 * Uses a timer to provide a short grace period for action
 * results and cancellation requests to be transmitted.
 *
 * @warning rclcpp::shutdown() stops the entire default ROS
 * context, not just this node.
 *
 * @note The delay does not guarantee downstream cancellation
 * acknowledgment.
 */
void ROLLGuidanceNode::request_shutdown() {
    if (shutdown_requested_.exchange(true)) {
        return;
    }

    if (this->get_parameter("test.disable_shutdown").as_bool()) {
        return;
    }

    spdlog::info("ROLL guidance terminating");

    shutdown_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        [this]() {
            shutdown_timer_->cancel();
            rclcpp::shutdown();
        });
}

/**
 * @brief Executes an accepted ROLL guidance action.
 *
 * Waits until the AUV reaches the configured trigger distance,
 * then generates orientation waypoints and sends them to the
 * Waypoint Manager.
 *
 * The ROLL action terminates when either:
 * - The Waypoint Manager finishes the manoeuver.
 * - The AUV passes the gate.
 * - The action is canceled or fails.
 *
 * @param goal_handle Handle of the accepted ROLL action goal.
 */
void ROLLGuidanceNode::execute(
    const std::shared_ptr<GoalHandleROLL> goal_handle) {

    {
        std::lock_guard<std::mutex> lock(mutex_);

        goal_handle_ = goal_handle;
        goal_start_time_ = SteadyClock::now();

        maneuver_started_ = false;

        roll_tracking_active_ = false;
        roll_tracking_error_ = false;
    }

    spdlog::info("Executing ROLL guidance goal");

    rclcpp::Rate loop_rate(1000.0 / time_step_.count());

    while (rclcpp::ok() && !finishing_.load()) {

        if (goal_handle->is_canceling()) {
            //the watchdog will cancel the Waypoint Manager goal
            //and finish the ROLL action.
            return;
        }

        vortex::utils::types::Pose initial_pose;
        bool ready_to_roll = false;
        bool already_passed = false;

        {
            std::lock_guard<std::mutex> lock(mutex_);

            already_passed = gate_passed_;

            if (!already_passed &&
                pose_received_ &&
                gate_received_ &&
                killswitch_received_ &&
                !killswitch_on_ &&
                should_start_roll(
                    current_pose_.pos_vector(),
                    gate_position_)) {

                initial_pose = current_pose_;
                ready_to_roll = true;
            }
        }

        if (already_passed) {
            finish_roll(
                goal_handle,
                FinishReason::GatePassed);
            return;
        }

        if (!ready_to_roll) {
            loop_rate.sleep();
            continue;
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);

            if (finishing_.load() ||
                cancel_requested_ ||
                goal_handle->is_canceling() ||
                !killswitch_received_ ||
                killswitch_on_) {

                return;
            }
        }
                
        spdlog::info("Starting ROLL manoeuver");

        {
            std::lock_guard<std::mutex> lock(mutex_);

            try {
                roll_initial_orientation_ =
                    initial_pose.ori_quaternion().normalized();

                roll_tracker_.reset(roll_initial_orientation_);

                roll_tracking_active_ = true;
                roll_tracking_error_ = false;

                maneuver_started_ = true;
                maneuver_start_time_ = SteadyClock::now();

            } catch (const std::exception& exception) {
                roll_tracking_active_ = false;
                roll_tracking_error_ = true;

                RCLCPP_ERROR(
                    this->get_logger(),
                    "Cannot initialise ROLL tracker: %s",
                    exception.what());
            }
        }

        bool tracker_failed = false;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            tracker_failed = roll_tracking_error_;
        }

        if (tracker_failed) {
            finish_roll(goal_handle, FinishReason::Failed);
            return;
        }

        const auto poses = roll_guidance_->generate_waypoints(initial_pose);

        WaypointManager::Goal waypoint_goal;
        waypoint_goal.convergence_threshold = 0.1;
        waypoint_goal.persistent = false;

        for (const auto& pose : poses) {
            vortex_msgs::msg::Waypoint waypoint;

            const auto position = pose.pos_vector();
            const auto orientation = pose.ori_quaternion();

            waypoint.pose.position.x = position.x();
            waypoint.pose.position.y = position.y();
            waypoint.pose.position.z = position.z();

            waypoint.pose.orientation.w = orientation.w();
            waypoint.pose.orientation.x = orientation.x();
            waypoint.pose.orientation.y = orientation.y();
            waypoint.pose.orientation.z = orientation.z();

            waypoint.waypoint_mode.mode =
                vortex_msgs::msg::WaypointMode::FULL_POSE;

            waypoint.keep_altitude = false;
            waypoint.require_altitude_convergence = false;

            waypoint_goal.waypoints.push_back(waypoint);
        }

        if (!waypoint_manager_client_->action_server_is_ready()) {
            spdlog::error("Waypoint Manager is not ready");

            finish_roll(
                goal_handle,
                FinishReason::Failed);
            return;
        }

        using SendGoalOptions =
            rclcpp_action::Client<WaypointManager>::SendGoalOptions;

        SendGoalOptions options;

        options.goal_response_callback =
            [this, goal_handle](
                std::shared_future<
                    WaypointManagerGoalHandle::SharedPtr> future) {

                auto manager_handle = future.get();

                if (!manager_handle) {
                    spdlog::error(
                        "Waypoint Manager rejected ROLL goal");

                    finish_roll(
                        goal_handle,
                        FinishReason::Failed);
                    return;
                }

                bool should_cancel = false;

                {
                    std::lock_guard<std::mutex> lock(mutex_);

                    should_cancel =
                        finishing_.load() ||
                        goal_handle->is_canceling();

                    if (!should_cancel) {
                        waypoint_manager_goal_handle_ =
                            manager_handle;
                    }
                }

                if (should_cancel) {
                    waypoint_manager_client_->async_cancel_goal(
                        manager_handle);
                }
            };

        options.result_callback =
            [this, goal_handle](
                const WaypointManagerGoalHandle::WrappedResult&
                    manager_result) {

                if (finishing_.load()) {
                    return;
                }

                const bool succeeded =
                    manager_result.code ==
                        rclcpp_action::ResultCode::SUCCEEDED &&
                    manager_result.result &&
                    manager_result.result->success;

                if (goal_handle->is_canceling()) {
                    finish_roll(
                        goal_handle,
                        FinishReason::Canceled);
                } else if (succeeded) {

                    bool verified = false;
                    double measured_roll = 0.0;

                    {
                        std::lock_guard<std::mutex> lock(mutex_);

                        verified = roll_verified();

                        measured_roll =
                            roll_tracker_.accumulated_roll();
                    }

                    if (verified) {
                        RCLCPP_INFO(
                            get_logger(),
                            "ROLL verified: measured %.2f degrees",
                            measured_roll * 180.0 / M_PI);

                        finish_roll(
                            goal_handle,
                            FinishReason::Completed);

                    } else {
                        RCLCPP_ERROR(
                            get_logger(),
                            "Waypoint Manager succeeded but ROLL "
                            "verification failed (measured %.2f degrees)",
                            measured_roll * 180.0 / M_PI);

                        finish_roll(
                            goal_handle,
                            FinishReason::Failed);
                    } else {
                    spdlog::error("ROLL manoeuver failed");

                    finish_roll(
                        goal_handle,
                        FinishReason::Failed);
                }
            };

        if (finishing_.load()) {
            return;
        }

        waypoint_manager_client_->async_send_goal(
            waypoint_goal,
            options);

        //the result callback or gate crossing finishes the action.
        return;
    }

    if (!finishing_.load() && goal_handle->is_active()) {
        finish_roll(
            goal_handle,
            FinishReason::Failed);
    }
}

}  // namespace vortex::guidance