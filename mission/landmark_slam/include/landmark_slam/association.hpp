#ifndef LANDMARK_SLAM__ASSOCIATION_HPP_
#define LANDMARK_SLAM__ASSOCIATION_HPP_

#include <cstddef>
#include <optional>
#include <vector>

#include "landmark_slam/graph.hpp"

namespace vortex::landmark_slam {

/// One detection of a batch (one detector message), at keyframe kf.
struct Detection {
    const ClassConfig* cls{nullptr};
    Measurement z;
};

struct Match {
    std::size_t detection{0};
    int landmark{0};
    /// Normalised innovation squared of the pair, d^2 / dof.
    double nis{0.0};
};

struct Association {
    std::vector<Match> matches;
    /// Detections compatible with no landmark (candidates for new ones).
    std::vector<std::size_t> unmatched;
};

/// chi^2 quantile for this probability and degrees of freedom.
double chi2_threshold(double prob, int dof);

/**
 * @brief Joint Compatibility Branch and Bound (Neira & Tardos 2001) with the
 * class as a hard constraint. Every pair is gated on its own (individual
 * compatibility), then the largest set of pairs that passes the joint
 * chi^2 test is kept (ties: lowest joint Mahalanobis distance). The
 * covariances include the cross-covariance between the keyframe and the
 * landmarks. A match that another candidate explains almost as well is
 * ambiguous and dropped. The graph must be updated with keyframe kf.
 */
Association associate(const LandmarkGraph& graph,
                      int kf,
                      const std::vector<Detection>& detections);

/// A detection no landmark explains: a vote for a new landmark.
struct Vote {
    int kf{0};
    Measurement z;
    /// Where the detection puts the object in the map frame when cast.
    gtsam::Point3 point;
    /// Object yaw in the map frame, if the detector measured it.
    std::optional<double> yaw;
    double t{0.0};
};

/// Votes strong enough to become a landmark.
struct VoteCluster {
    ClassConfig cls;
    /// Weighted centroid of the votes.
    gtsam::Point3 position;
    std::optional<double> yaw;
    std::vector<Vote> votes;
};

/**
 * @brief Landmark initialisation by voting. Every detection that no
 * landmark explains is a vote at its map position, per class; a vote's
 * weight halves every kVoteHalfLifeS. Where the vote weight within
 * vote_radius_m reaches min_votes, those votes become a landmark at their
 * weighted centroid. Votes at an existing landmark of the class (within
 * vote_radius_m) are a second look at it and are dropped; for a class in
 * the prior map, votes far from every prior entry are rejected.
 */
class Votes {
   public:
    void clear() { classes_.clear(); }
    void add(const Detection& d,
             int kf,
             const gtsam::Pose3& T_map_base,
             double t,
             const Config& cfg);
    /// Remove and return the clusters that become landmarks now.
    std::vector<VoteCluster> take_clusters(const LandmarkGraph& graph,
                                           double t);

   private:
    struct ClassVotes {
        ClassConfig cls;
        std::vector<Vote> votes;
    };
    std::vector<ClassVotes> classes_;
};

}  // namespace vortex::landmark_slam

#endif  // LANDMARK_SLAM__ASSOCIATION_HPP_
