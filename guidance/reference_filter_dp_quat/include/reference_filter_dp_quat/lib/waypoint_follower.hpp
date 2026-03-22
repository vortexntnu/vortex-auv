#ifndef REFERENCE_FILTER_DP_QUAT__LIB__WAYPOINT_FOLLOWER_HPP_
#define REFERENCE_FILTER_DP_QUAT__LIB__WAYPOINT_FOLLOWER_HPP_

#include <mutex>
#include <vortex/utils/types.hpp>
#include "reference_filter_dp_quat/lib/eigen_typedefs.hpp"
#include "reference_filter_dp_quat/lib/reference_filter.hpp"
#include "reference_filter_dp_quat/lib/waypoint_types.hpp"

namespace vortex::guidance {

using vortex::utils::types::Pose;
using vortex::utils::types::Twist;

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
     * @param convergence_threshold Max error norm to consider target reached.
     */
    void start(const Pose& pose,
               const Twist& twist,
               const Waypoint& waypoint,
               double convergence_threshold);

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
     * @return True if the error norm is within the convergence threshold.
     */
    bool within_convergance(const Pose& measured_pose) const;

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
    double convergence_threshold_{0.1};
};

}  // namespace vortex::guidance

#endif  // REFERENCE_FILTER_DP_QUAT__LIB__WAYPOINT_FOLLOWER_HPP_
