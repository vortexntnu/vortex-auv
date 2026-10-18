#ifndef REFERENCE_FILTER_DP_QUAT__LIB__WAYPOINT_FOLLOWER_HPP_
#define REFERENCE_FILTER_DP_QUAT__LIB__WAYPOINT_FOLLOWER_HPP_

#include <mutex>
#include <optional>
#include <vortex/utils/types.hpp>
#include <vortex/utils/waypoint_utils.hpp>
#include "reference_filter_dp_quat/lib/eigen_typedefs.hpp"
#include "reference_filter_dp_quat/lib/reference_filter.hpp"

namespace vortex::guidance {

using vortex::utils::types::Pose;
using vortex::utils::types::Twist;
using vortex::utils::types::Waypoint;
using vortex::utils::types::WaypointMode;
using vortex::utils::waypoints::ConvergenceTolerance;

/**
 * @brief Manages reference filter state and waypoint following logic.
 *
 * Uses an error-state formulation: the 18D state vector holds
 * [δp(3), δφ(3), velocity(6), acceleration(6)], while a separate
 * nominal Pose tracks the full quaternion orientation. Each step,
 * position/orientation errors are absorbed into the nominal and reset.
 *
 * Thread-safe: all public methods acquire a mutex.
 */
class WaypointFollower {
   public:
    WaypointFollower(const ReferenceFilterParams& params, double dt_seconds);

    /**
     * @brief Initialize the follower with the current vehicle state and target.
     * @param pose Current vehicle pose.
     * @param twist Current vehicle twist (body frame).
     * @param waypoint Target waypoint with mode.
     * @param tolerance Position and orientation tolerances to consider target
     * reached.
     */
    void start(const Pose& pose,
               const Twist& twist,
               const Waypoint& waypoint,
               const ConvergenceTolerance& tolerance);

    /**
     * @brief Change the target and tolerances while keeping the filter state.
     * @param waypoint New target waypoint with mode.
     * @param tolerance Position and orientation tolerances to consider target
     * reached.
     */
    void retarget(const Waypoint& waypoint,
                  const ConvergenceTolerance& tolerance);

    /**
     * @brief Advance the filter by one time step.
     *
     * Computes the error-state reference, integrates, then absorbs
     * position/orientation errors into the nominal pose.
     */
    void step();

    /**
     * @brief Check if the measured pose has converged to the waypoint goal.
     * @param measured_pose Current measured pose.
     * @return True if the error norm is within the tolerances.
     */
    bool within_convergance(const Pose& measured_pose) const;

    /**
     * @brief Check convergence, requiring the tolerances to hold continuously.
     *
     * The hold timer resets whenever the pose leaves the tolerances, and on
     * start() and retarget().
     * @param measured_pose Current measured pose.
     * @param t_sec Current time in seconds.
     * @param hold_time_sec Time the tolerances must hold; <= 0 converges on
     * the first sample within the tolerances.
     * @return True if the tolerances have held for hold_time_sec.
     */
    bool update_convergence(const Pose& measured_pose,
                            double t_sec,
                            double hold_time_sec);

    /**
     * @brief Update the reference goal pose mid-sequence.
     * @param reference_goal_pose The new reference pose.
     */
    void set_reference(const Pose& reference_goal_pose);

    /**
     * @brief Snap the nominal pose to the waypoint goal and zero
     * errors/velocity.
     *
     * Useful after convergence to eliminate any remaining steady-state offset.
     */
    void snap_state_to_reference();

    /**
     * @brief Get the current nominal pose (position + quaternion).
     */
    Pose pose() const;

    /**
     * @brief Get the current world-frame velocity (linear + angular).
     */
    Eigen::Vector6d velocity() const;

    /**
     * @brief Get the current waypoint goal.
     */
    Pose waypoint_goal() const;

   private:
    /// @brief Convergence check for callers that already hold the mutex.
    bool within_locked(const Pose& measured_pose) const;

    /**
     * @brief Compute error-state reference based on the current nominal pose
     * and waypoint goal.
     */
    Eigen::Vector6d update_reference() const;

    /**
     * @brief Absorb position/orientation errors into the nominal pose and
     * reset the error states.
     */
    void inject_and_reset();

    mutable std::mutex mutex_;
    ReferenceFilter filter_;
    double dt_seconds_{0.01};
    Pose nominal_pose_;
    Eigen::Vector18d state_ = Eigen::Vector18d::Zero();
    Pose waypoint_goal_;
    WaypointMode waypoint_mode_{WaypointMode::FULL_POSE};
    ConvergenceTolerance tolerance_{};
    std::optional<double> inside_since_sec_;
};

}  // namespace vortex::guidance

#endif  // REFERENCE_FILTER_DP_QUAT__LIB__WAYPOINT_FOLLOWER_HPP_
