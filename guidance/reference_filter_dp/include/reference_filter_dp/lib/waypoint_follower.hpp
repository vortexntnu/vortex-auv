#ifndef REFERENCE_FILTER_DP__LIB__WAYPOINT_FOLLOWER_HPP_
#define REFERENCE_FILTER_DP__LIB__WAYPOINT_FOLLOWER_HPP_

#include <mutex>
#include <optional>
#include <vortex/utils/types.hpp>
#include "reference_filter_dp/lib/eigen_typedefs.hpp"
#include "reference_filter_dp/lib/reference_filter.hpp"
#include "reference_filter_dp/lib/waypoint_types.hpp"

namespace vortex::guidance {

using vortex::utils::types::PoseEuler;
using vortex::utils::types::Twist;

/**
 * @brief Manages reference filter state and waypoint following logic.
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
    void start(const PoseEuler& pose,
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
     * @return  The updated filter state.
     */
    Eigen::Vector18d step();

    /**
     * @brief Check if the measured pose has converged to the reference goal.
     * @param measured_pose Current measured pose.
     * @return True if the error norm is within the tolerances.
     */
    bool within_convergance(const Eigen::Vector6d& measured_pose) const;

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
    bool update_convergence(const Eigen::Vector6d& measured_pose,
                            double t_sec,
                            double hold_time_sec);

    /**
     * @brief Update the reference goal pose mid-sequence.
     * @param reference_goal_pose The new reference pose.
     */
    void set_reference(const PoseEuler& reference_goal_pose);

    /**
     * @brief Snap the position component of the filter state to the reference.
     *
     * Useful after convergence to eliminate any remaining steady-state offset
     */
    void snap_state_to_reference();

    /// @brief Get the current 18D filter state.
    Eigen::Vector18d state() const;

    /// @brief Get the current 6D reference goal pose.
    Eigen::Vector6d reference() const;

   private:
    /// @brief Convergence check for callers that already hold the mutex.
    bool within_locked(const Eigen::Vector6d& measured_pose) const;

    Eigen::Vector18d compute_initial_state(const PoseEuler& pose,
                                           const Twist& twist);

    mutable std::mutex mutex_;
    ReferenceFilter filter_;
    double dt_seconds_{0.01};
    Eigen::Vector18d state_ = Eigen::Vector18d::Zero();
    Eigen::Vector6d reference_goal_ = Eigen::Vector6d::Zero();
    WaypointMode waypoint_mode_{WaypointMode::FULL_POSE};
    ConvergenceTolerance tolerance_{};
    std::optional<double> inside_since_sec_;
};

}  // namespace vortex::guidance

#endif  // REFERENCE_FILTER_DP__LIB__WAYPOINT_FOLLOWER_HPP_
