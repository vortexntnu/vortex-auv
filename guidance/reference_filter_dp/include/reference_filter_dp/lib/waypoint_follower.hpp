#ifndef REFERENCE_FILTER_DP__LIB__WAYPOINT_FOLLOWER_HPP_
#define REFERENCE_FILTER_DP__LIB__WAYPOINT_FOLLOWER_HPP_

#include <mutex>
#include <vortex/utils/types.hpp>
#include "reference_filter_dp/lib/reference_filter.hpp"
#include "reference_filter_dp/lib/waypoint_types.hpp"

namespace vortex::guidance {

using vortex::utils::types::PoseEuler;
using vortex::utils::types::Twist;

/**
 * @brief Result of a single step of the waypoint follower.
 */
struct StepResult {
    Eigen::Vector18d
        reference_state;  ///< Filter state: [pose, pose_dot, pose_ddot].
    bool target_reached;  ///< True if measured pose has converged.
};

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
     * @param convergence_threshold Max error norm to consider target reached.
     */
    void start(const PoseEuler& pose,
               const Twist& twist,
               const Waypoint& waypoint,
               double convergence_threshold);

    /**
     * @brief Advance the filter by one time step.
     * @param measured_pose Current measured pose for convergence checking.
     * @return StepResult with the updated filter state and convergence status.
     */
    StepResult step(const Eigen::Vector6d& measured_pose);

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
    Eigen::Vector18d compute_initial_state(const PoseEuler& pose,
                                           const Twist& twist);

    mutable std::mutex mutex_;
    ReferenceFilter filter_;
    double dt_seconds_{0.01};
    Eigen::Vector18d state_ = Eigen::Vector18d::Zero();
    Eigen::Vector6d reference_goal_ = Eigen::Vector6d::Zero();
    WaypointMode waypoint_mode_{WaypointMode::FULL_POSE};
    double convergence_threshold_{0.1};
};

}  // namespace vortex::guidance

#endif  // REFERENCE_FILTER_DP__LIB__WAYPOINT_FOLLOWER_HPP_
