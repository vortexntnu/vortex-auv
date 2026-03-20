#ifndef REFERENCE_FILTER_DP__LIB__WAYPOINT_FOLLOWER_HPP_
#define REFERENCE_FILTER_DP__LIB__WAYPOINT_FOLLOWER_HPP_

#include <vortex/utils/types.hpp>
#include "reference_filter_dp/lib/reference_filter.hpp"
#include "reference_filter_dp/lib/waypoint_types.hpp"

namespace vortex::guidance {

using vortex::utils::types::PoseEuler;
using vortex::utils::types::Twist;

struct StepResult {
    Eigen::Vector18d state;
    Eigen::Vector6d reference;
    bool converged;
};

class WaypointFollower {
   public:
    WaypointFollower(const ReferenceFilterParams& params, double dt_seconds);

    void start(const PoseEuler& pose,
               const Twist& twist,
               const Waypoint& waypoint,
               double convergence_threshold);

    StepResult step(const Eigen::Vector6d& measured_pose);

    void set_reference(const PoseEuler& reference_goal_pose, WaypointMode mode);

    const Eigen::Vector18d& state() const;
    const Eigen::Vector6d& reference() const;
    void snap_state_to_reference();

   private:
    Eigen::Vector18d compute_initial_state(const PoseEuler& pose,
                                           const Twist& twist);

    ReferenceFilter filter_;
    double dt_seconds_;
    Eigen::Vector18d x_ = Eigen::Vector18d::Zero();
    Eigen::Vector6d reference_goal_ = Eigen::Vector6d::Zero();
    Waypoint waypoint_{};
    double convergence_threshold_{0.1};
};

}  // namespace vortex::guidance

#endif  // REFERENCE_FILTER_DP__LIB__WAYPOINT_FOLLOWER_HPP_
