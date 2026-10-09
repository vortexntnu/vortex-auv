#ifndef LANDMARK_SERVER__ASSOCIATION_HPP_
#define LANDMARK_SERVER__ASSOCIATION_HPP_

#include <cstddef>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "landmark_server/graph.hpp"

namespace vortex::landmark_server {

/// One detection of a message, in the base frame of keyframe kf.
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
    /// Detections no landmark could explain (inside no gate): they may be
    /// a new object. A detection inside a gate that is not matched (lost to
    /// another detection, or ambiguous) is neither.
    std::vector<std::size_t> unmatched;
};

/// chi^2 quantile for this probability and degrees of freedom.
double chi2_threshold(double prob, int dof);

/**
 * @brief Global nearest neighbour per class: the squared Mahalanobis
 * distance of each detection-landmark pair (innovation covariance from the
 * joint marginal of the keyframe and the landmark, cross-covariance
 * included) is the cost, the chi^2 gate at gate_prob the limit, the
 * Hungarian algorithm the one-to-one assignment. A match that another
 * landmark explains nearly as well (d^2 within ambiguity_d2) is dropped.
 *
 * Every landmark in the map takes part, not only recently seen ones: a
 * remembered landmark seen again after a loop is matched under its id and
 * closes the loop. The graph must be updated with keyframe kf.
 */
Association associate(const LandmarkGraph& graph,
                      int kf,
                      const std::vector<Detection>& detections);

/// Minimum-cost assignment of rows to columns (rows <= cols), for each row
/// the column.
std::vector<int> solve_assignment(const Eigen::MatrixXd& cost);

/// One detection of a candidate.
struct Hit {
    int kf{0};
    Measurement z;
    gtsam::Point3 point;  // map frame
    std::optional<double> yaw;
    double t{0.0};
};

/// A possible new landmark: detections of one class close together.
struct Candidate {
    const ClassConfig* cls{nullptr};
    gtsam::Point3 position;  // mean of the hits
    std::vector<Hit> hits;
};

/**
 * @brief New landmarks. A detection no landmark explains joins the nearest
 * candidate of its class within candidate_radius_m (one detection per
 * candidate and message), or starts one. A candidate with confirm_hits hits
 * within confirm_window_s becomes a landmark; one without a hit for
 * confirm_window_s is forgotten. A class with a prior entry only takes
 * detections within prior_radius_m of it.
 */
class Candidates {
   public:
    void clear() { candidates_.clear(); }
    /// The unmatched detections of one message.
    void add(const std::vector<Detection>& detections,
             const std::vector<std::size_t>& unmatched,
             int kf,
             const gtsam::Pose3& T_map_base,
             double t,
             const Params& params,
             const std::map<std::string, gtsam::Point3>& priors);
    /// Remove and return the candidates confirmed now. A confirmed
    /// candidate at a landmark of its class (within merge_radius_m) is a
    /// second look at it and dropped.
    std::vector<Candidate> take_confirmed(const LandmarkGraph& graph, double t);
    std::size_t size() const { return candidates_.size(); }

   private:
    std::vector<Candidate> candidates_;
};

}  // namespace vortex::landmark_server

#endif  // LANDMARK_SERVER__ASSOCIATION_HPP_
