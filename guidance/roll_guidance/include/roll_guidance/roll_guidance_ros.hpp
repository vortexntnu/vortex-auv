#ifndef ROLL_GUIDANCE__ROLL_GUIDANCE_ROS_HPP_
#define ROLL_GUIDANCE__ROLL_GUIDANCE_ROS_HPP_

#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <std_msgs/msg/bool.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <vortex_msgs/action/guidance_waypoint.hpp>
#include <vortex_msgs/action/waypoint_manager.hpp>
#include <vortex_msgs/msg/waypoint.hpp>
#include <vortex_msgs/msg/waypoint_mode.hpp>

#include "roll_guidance/roll_guidance.hpp"
#include "roll_guidance/roll_tracker.hpp"

namespace vortex::guidance {

/**
 * @brief ROS 2 node for executing an AUV roll maneuver.
 *
 * Monitors the AUV pose and gate position, starts a roll maneuver
 * when the AUV reaches the configured trigger distance, and sends
 * generated orientation waypoints to the Waypoint Manager.
 *
 * The action terminates when the maneuver completes, the gate
 * is crossed, the action is canceled, or an error occurs.
 */
class ROLLGuidanceNode : public rclcpp::Node {
   public:
    /**
     * @brief Constructs the ROLL guidance ROS node.
     * @param options ROS 2 node options.
     */
    explicit ROLLGuidanceNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    /**
     * @brief Indicates whether ROLL has requested node shutdown.
     *
     * Used by integration tests to verify shutdown behavior
     * without terminating the shared ROS 2 context.
     *
     * @return true if shutdown was requested.
     */
    bool shutdown_was_requested() const {
        return shutdown_requested_.load();
    }

   private:
    /**
     * @brief Class aliases for ROS 2 actions and goal handles.
     */
    using GuidanceWaypoint = vortex_msgs::action::GuidanceWaypoint;

    using GoalHandleROLL =
        rclcpp_action::ServerGoalHandle<GuidanceWaypoint>;

    using WaypointManager =
        vortex_msgs::action::WaypointManager;

    using WaypointManagerGoalHandle =
        rclcpp_action::ClientGoalHandle<WaypointManager>;

    /**
     * @brief Reasons for terminating the ROLL guidance action.
     */
    enum class FinishReason {
        Completed,
        GatePassed,
        Canceled,
        Failed
    };

    /**
     * @brief Sets up the subscribers.
     */
    void set_subscribers();

    /**
     * @brief Sets up the ROLL guidance action server.
     */
    void set_action_server();

    /**
     * @brief Sets up the Waypoint Manager action client.
     */
    void set_waypoint_manager_client();

    /**
     * @brief Initialises the ROLL manoeuvre generator and its parameters.
     */
    void set_roll_guidance();

    /**
     * @brief Receives the current AUV pose.
     * @param msg Current AUV pose with covariance.
     */
    void pose_callback(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    /**
     * @brief Receives the gate waypoint from a ROS topic.
     * @param msg Gate waypoint position with timestamp and reference frame.
     */
    void waypoint_callback(
        const geometry_msgs::msg::PointStamped::SharedPtr msg);
    
    /**
     * @brief Receives software killswitch status.
     * @param msg True when the killswitch is engaged.
     */
    void killswitch_callback(
        const std_msgs::msg::Bool::SharedPtr msg);

    /**
     * @brief Checks pose freshness, timeouts, and cancellation.
     */
    void watchdog_callback();

    /**
     * @brief Handles an incoming ROLL guidance action goal request.
     * @param uuid Unique identifier for the received goal.
     * @param goal Requested ROLL guidance goal.
     * @return rclcpp_action::GoalResponse Response indicating whether
     * the goal is accepted.
     */
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const GuidanceWaypoint::Goal> goal);

    /**
     * @brief Handles cancellation of an active ROLL guidance goal.
     * @param goal_handle Handle to the goal being cancelled.
     * @return rclcpp_action::CancelResponse Response indicating whether
     * cancellation is accepted.
     */
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleROLL> goal_handle);

    /**
     * @brief Handles an accepted ROLL guidance goal.
     * @param goal_handle Handle to the accepted goal.
     */
    void handle_accepted(
        const std::shared_ptr<GoalHandleROLL> goal_handle);

    /**
     * @brief Executes the ROLL guidance action.
     *
     * Monitors the AUV's distance to the gate and starts the roll
     * maneuver once the configured trigger distance is reached.
     *
     * Once triggered, sends sequential orientation waypoints to the
     * Waypoint Manager and waits for the maneuver result.
     *
     * @param goal_handle Handle to the active ROLL guidance goal.
     */
    void execute(
        const std::shared_ptr<GoalHandleROLL> goal_handle);

    /**
     * @brief Checks whether the AUV is close enough to the gate
     * to start the roll maneuver.
     *
     * @param current_position Current AUV position.
     * @param gate_position Gate position in the same reference frame.
     * @return true if the AUV is within the trigger distance,
     * false otherwise.
     */
    bool should_start_roll(
        const Eigen::Vector3d& current_position,
        const Eigen::Vector3d& gate_position) const;

    /**
     * @brief Checks whether the AUV has crossed the gate plane.
     *
     * Calculates the signed distance between the AUV and the gate
     * along the configured gate normal.
     *
     * The AUV must first be observed on the approach side before
     * crossing to the opposite side.
     *
     * @param position Current AUV position in the gate's reference frame.
     * @return true if the AUV has crossed the gate plane,
     * false otherwise.
     *
     * @note Must be called while holding mutex_.
     */
    bool has_passed_gate(const Eigen::Vector3d& position);

    /**
     * @brief Finalises the ROLL guidance action.
     *
     * Ensures that only one event can finish the action.
     * If the gate is passed before the roll completes, requests
     * cancellation of the Waypoint Manager goal.
     *
     * Completes the ROLL action according to the specified reason
     * and initiates node shutdown.
     *
     * @param goal_handle Handle of the active ROLL action goal.
     * @param reason Reason why the ROLL action is ending.
     *
     * @note Must be called without holding mutex_.
     */
    void finish_roll(
        const std::shared_ptr<GoalHandleROLL>& goal_handle,
        FinishReason reason);

    /**
     * @brief Requests shutdown of the ROLL guidance node.
     *
     * Schedules a one-time shutdown after a short delay,
     * allowing pending ROS 2 action messages to be transmitted.
     *
     * @note Calling rclcpp::shutdown() stops the entire ROS 2
     * context, not only this node.
     */
    void request_shutdown();
    
    /**
     * @brief ROS interface for the ROLL guidance action server.
     */
    rclcpp_action::Server<GuidanceWaypoint>::SharedPtr action_server_;

    rclcpp_action::Client<WaypointManager>::SharedPtr
        waypoint_manager_client_;

    std::shared_ptr<WaypointManagerGoalHandle>
        waypoint_manager_goal_handle_;

    rclcpp::Subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
        pose_sub_;

    rclcpp::Subscription<
        geometry_msgs::msg::PointStamped>::SharedPtr
        waypoint_sub_;

    rclcpp::CallbackGroup::SharedPtr cb_group_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
        killswitch_sub_;

    /**
     * @brief Indicates whether a killswitch message has been received.
     */
    bool killswitch_received_{false};

    /**
     * @brief Indicates whether the killswitch is currently engaged.
     */
    bool killswitch_on_{true};

    /**
     * @brief Timeouts for pose, trigger, and manoeuver completion.
     */
    double pose_timeout_seconds_{0.5};

    /**
     * @brief Timeouts for trigger and manoeuver completion.
     */
    double trigger_timeout_seconds_{30.0};

    /**
     * @brief Timeouts for manoeuver completion.
     */
    double maneuver_timeout_seconds_{120.0};

    /**
     * @brief Current gate position in the same reference frame as the AUV pose.
     */
    Eigen::Vector3d gate_position_{Eigen::Vector3d::Zero()};

    /**
     * @brief Indicates whether a gate waypoint has been received.
     */
    bool gate_received_{false};

    /**
     * @brief Normal vector of the gate plane.
     *
     * Defines the direction in which the AUV must cross the gate.
     * The vector must be normalised before use.
     */
    Eigen::Vector3d gate_normal_{1.0, 0.0, 0.0};

    /**
     * @brief Indicates whether the AUV has been observed
     * on the approach side of the gate.
     */
    bool gate_approach_seen_{false};

    /**
     * @brief Indicates whether the AUV has crossed the gate plane.
     */
    bool gate_passed_{false};

    //guidance
    std::unique_ptr<RollManeuverGenerator> roll_guidance_;
    ROLL::Params params_;

    //current vehicle state
    vortex::utils::types::Pose current_pose_;
    bool pose_received_{false};

    /**
     * @brief Verifies the measured roll and final AUV orientation.
     *
     * @return true if the requested rotation and final orientation
     * have both converged within their configured tolerances.
     *
     * @note Must be called while holding mutex_.
     */
    bool roll_verified() const;

    //timing
    std::chrono::milliseconds time_step_{10};

    //action state
    std::mutex mutex_;

    /**
     * @brief Handle the currently active ROLL action goal.
     */
    std::shared_ptr<GoalHandleROLL> goal_handle_;

    /**
     * @brief Prevents the ROLL action from being finalised more than once.
     *
     * This atomic flag ensures that only the first terminal event
     * can initiate action completion.
     */
    std::atomic<bool> finishing_{false};

    /**
     * @brief Indicates whether node shutdown has already been requested.
     *
     * Prevents multiple shutdown requests from being scheduled.
     */
    std::atomic<bool> shutdown_requested_{false};

    /**
     * @brief Indicates whether node cancellation has already been requested.
     */
    bool cancel_requested_{false};

    /**
     * @brief Tracks actual signed roll measured from the AUV pose.
     */
    RollTracker roll_tracker_;

    /**
     * @brief Indicates that the roll tracker is currently active.
     */
    bool roll_tracking_active_{false};

    /**
     * @brief Indicates that a roll measurement was rejected.
     */
    bool roll_tracking_error_{false};
    

    /**
     * @brief Maximum difference between requested and measured
     * total roll at completion, in radians.
     */
    double roll_tolerance_{0.0872664626}; //5 degrees

    /**
     * @brief Maximum allowed final orientation error in radians.
     */
    double orientation_tolerance_{0.0872664626};

    /**
     * @brief Initial orientation captured when the roll starts.
     */
    Eigen::Quaterniond roll_initial_orientation_{
        Eigen::Quaterniond::Identity()};

    /**
     * @brief Indicates whether a ROLL action goal has been claimed.
     *
     * @note The existing node supports one maneuver per execution.
     */
    std::atomic<bool> goal_claimed_{false};

    /**
     * @brief Indicates whether a ROLL action goal has already been accepted.
     *
     * Prevents multiple ROLL goals from being accepted by the same
     * node instance.
     *
     * @note This flag is intentionally not reset because the current
     * ROLL guidance node is designed for one maneuver per execution.
     */
    std::atomic<bool> goal_claimed_{false};

    /**
     * @brief Timer used to delay ROS 2 shutdown.
     *
     * Provides a short grace period for pending action results
     * and cancellation requests to be transmitted.
     */
    rclcpp::TimerBase::SharedPtr shutdown_timer_;
};

}  // namespace vortex::guidance

#endif  // ROLL_GUIDANCE__ROLL_GUIDANCE_ROS_HPP_