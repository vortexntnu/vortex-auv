#ifndef REFERENCE_FILTER_DP__LIB__WAYPOINT_FOLLOWER_HPP_
#define REFERENCE_FILTER_DP__LIB__WAYPOINT_FOLLOWER_HPP_

#include <mutex>
#include <vortex/utils/types.hpp>
#include "reference_filter_dp/lib/reference_filter.hpp"
#include "reference_filter_dp/lib/waypoint_types.hpp"

namespace vortex::guidance {

using vortex::utils::types::PoseEuler;
using vortex::utils::types::Twist;

struct StepResult {
    Eigen::Vector18d reference_state;
    bool target_reached;
};

class WaypointFollower {
   public:
    WaypointFollower(const ReferenceFilterParams& params, double dt_seconds);

    void start(const PoseEuler& pose,
               const Twist& twist,
               const Waypoint& waypoint,
               double convergence_threshold);

    StepResult step(const Eigen::Vector6d& measured_pose);

    void set_reference(const PoseEuler& reference_goal_pose);

    void snap_state_to_reference();

    Eigen::Vector18d state() const;
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
