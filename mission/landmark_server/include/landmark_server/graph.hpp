#ifndef LANDMARK_SERVER__GRAPH_HPP_
#define LANDMARK_SERVER__GRAPH_HPP_

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sam/BearingRangeFactor.h>

#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "landmark_server/config.hpp"

namespace vortex::landmark_server {

inline gtsam::Key X(int kf) {
    return gtsam::Symbol('x', static_cast<std::uint64_t>(kf));
}
inline gtsam::Key L(int id) {
    return gtsam::Symbol('l', static_cast<std::uint64_t>(id));
}

using BearingRange = gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Pose3>;

/// A detection in the base frame of the keyframe it is attached to.
struct Measurement {
    gtsam::Point3 position;
    /// Object orientation relative to the base, when the detector gives one.
    std::optional<gtsam::Rot3> rotation;
    /// Detections averaged into position: the factor's noise is divided by
    /// sqrt(merged), merged at most max_merged_per_factor (they share the
    /// view's errors).
    std::size_t merged{1};
};

/// What the map knows about one landmark.
struct LandmarkState {
    int id{0};
    ClassConfig cls;
    gtsam::Pose3 pose;
    /// Marginal covariance in the landmark's tangent space (GTSAM order:
    /// rotation, then translation, both in the landmark frame).
    gtsam::Matrix6 cov{gtsam::Matrix6::Identity()};
    /// Position covariance relative to the vehicle (latest keyframe), map
    /// axes: what navigating to it depends on.
    gtsam::Matrix3 relative_cov{gtsam::Matrix3::Identity()};
    int n_obs{0};
    /// Detections per class of the landmark's group; cls is the majority.
    std::map<std::string, int> votes;
    double first_seen{0.0};
    double last_seen{0.0};
    /// Yaw is meaningful: from an orientation measurement.
    bool yaw_known{false};
};

/**
 * @brief iSAM2 graph of vehicle keyframes X(k) and landmarks L(id), both
 * Pose3 in the map frame. The map frame is the vehicle's pose at reset (the
 * start), levelled, at the pressure depth: x along the start heading, z down.
 *
 * Factors: odometry between keyframes (noise growing with the distance),
 * absolute depth and roll/pitch per keyframe, and detections: bearing-range,
 * or the relative pose when the detector gives an orientation and the class
 * has one. Detection factors use Dynamic Covariance Scaling so wrong matches
 * are down-weighted. A landmark seen again after a loop adds factors under
 * its id: that closes the loop.
 */
class LandmarkGraph {
   public:
    /// Clear everything; X(0) is the start.
    void reset(const Params& params, const gtsam::Pose3& T_odom_base, double t);
    /// New keyframe: odometry between factor, depth and roll/pitch.
    int add_keyframe(const gtsam::Pose3& T_odom_base, double t);
    void add_landmark(int id,
                      const ClassConfig& cls,
                      const gtsam::Pose3& init,
                      double t);
    void add_observation(int kf, int id, const Measurement& m, double t);
    /// One detection of the landmark was reported as this class of its
    /// group: the landmark's class becomes the one with the most votes.
    void vote(int id, const ClassConfig& cls);
    /// Stop using a landmark: no more observations, not in landmarks(). Its
    /// factors stay (they may be wrong ones, so it is not tied to another).
    void retire(int id);
    /// Of each pair of same-class landmarks closer than max_dist_m whose
    /// positions agree (Mahalanobis d^2 of the difference below d2_gate),
    /// the one seen less is retired. Returns the pairs (kept, retired).
    std::vector<std::pair<int, int>> retire_duplicates(double max_dist_m,
                                                       double d2_gate);
    /// isam.update() with what was added, refresh the estimates.
    void update();
    /// Refresh the landmark covariances (once per keyframe, to publish).
    void update_covariances();

    int last_keyframe() const {
        return static_cast<int>(keyframes_.size()) - 1;
    }
    double keyframe_time(int kf) const { return keyframes_.at(kf).t; }
    const gtsam::Pose3& keyframe_odom(int kf) const {
        return keyframes_.at(kf).T_odom_base;
    }
    gtsam::Pose3 keyframe_pose(int kf) const;
    gtsam::Pose3 landmark_pose(int id) const;
    /// The landmarks in use (not retired).
    const std::vector<LandmarkState>& landmarks() const { return cache_; }
    const Params& params() const { return params_; }
    /// map -> odom from the latest keyframe.
    gtsam::Pose3 map_to_odom() const;
    /// Joint marginal covariance (includes the cross-covariances).
    gtsam::JointMarginal joint_cov(const gtsam::KeyVector& keys) const;

   private:
    struct Keyframe {
        gtsam::Pose3 T_odom_base;
        double t{0.0};
    };
    void add_absolute_factors(int kf, const gtsam::Pose3& T_odom_base);
    gtsam::SharedNoiseModel robust(const gtsam::SharedNoiseModel& base) const;

    Params params_;
    std::unique_ptr<gtsam::ISAM2> isam_;
    gtsam::NonlinearFactorGraph new_factors_;
    gtsam::Values new_values_;
    gtsam::Values estimate_;
    mutable std::optional<gtsam::Marginals> marginals_;
    std::vector<Keyframe> keyframes_;
    std::map<int, LandmarkState> landmarks_;
    std::set<int> retired_;
    std::vector<LandmarkState> cache_;
};

/// Bearing-range noise sigmas [bearing, bearing, range] at this range.
gtsam::Vector3 bearing_range_sigmas(const Params& p, double range);

}  // namespace vortex::landmark_server

#endif  // LANDMARK_SERVER__GRAPH_HPP_
