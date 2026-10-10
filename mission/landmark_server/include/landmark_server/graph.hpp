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

/// A detection in the base frame of its keyframe.
struct Measurement {
    gtsam::Point3 position;
    std::optional<gtsam::Rot3> rotation;
    /// Number of detections averaged into this one.
    std::size_t merged{1};
};

struct LandmarkState {
    int id{0};
    ClassConfig cls;
    gtsam::Pose3 pose;
    /// GTSAM order: rotation, then translation, in the landmark frame.
    gtsam::Matrix6 cov{gtsam::Matrix6::Identity()};
    /// Position covariance relative to the latest keyframe, map axes.
    gtsam::Matrix3 relative_cov{gtsam::Matrix3::Identity()};
    int n_obs{0};
    /// Detections per class of the group. cls is the majority.
    std::map<std::string, int> votes;
    double first_seen{0.0};
    double last_seen{0.0};
    bool yaw_known{false};
};

/// iSAM2 graph of keyframes X(k) and landmarks L(id) in the map frame. The
/// map frame is the levelled start pose: x along the start heading, z down.
class LandmarkGraph {
   public:
    void reset(const Params& params, const gtsam::Pose3& T_odom_base, double t);
    int add_keyframe(const gtsam::Pose3& T_odom_base, double t);
    void add_landmark(int id,
                      const ClassConfig& cls,
                      const gtsam::Pose3& init,
                      double t);
    void add_observation(int kf, int id, const Measurement& m, double t);
    /// Counts a detection as this class. The landmark takes the majority.
    void vote(int id, const ClassConfig& cls);
    /// The landmark is no longer used or shown. Its factors stay.
    void retire(int id);
    /// Retires the less seen of two same-class landmarks at the same place.
    /// Returns the pairs (kept, retired).
    std::vector<std::pair<int, int>> retire_duplicates(double max_dist_m,
                                                       double d2_gate);
    void update();
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
    const std::vector<LandmarkState>& landmarks() const { return cache_; }
    const Params& params() const { return params_; }
    gtsam::Pose3 map_to_odom() const;
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

/// [bearing, bearing, range] sigmas at this range.
gtsam::Vector3 bearing_range_sigmas(const Params& p, double range);

}  // namespace vortex::landmark_server

#endif  // LANDMARK_SERVER__GRAPH_HPP_
