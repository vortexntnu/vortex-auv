#ifndef LANDMARK_SERVER__ASSOCIATION_HPP_
#define LANDMARK_SERVER__ASSOCIATION_HPP_

#include <cstddef>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "landmark_server/graph.hpp"

namespace vortex::landmark_server {

struct Detection {
    const ClassConfig* cls{nullptr};
    Measurement z;
};

struct Match {
    std::size_t detection{0};
    int landmark{0};
    double nis{0.0};
};

struct Association {
    std::vector<Match> matches;
    /// Detections outside every gate, so possibly a new object.
    std::vector<std::size_t> unmatched;
};

double chi2_threshold(double prob, int dof);

/// Matches detections to landmarks per class group: Mahalanobis cost, chi^2
/// gate, Hungarian assignment. Ambiguous matches are dropped. The graph must
/// already be updated with keyframe kf.
Association associate(const LandmarkGraph& graph,
                      int kf,
                      const std::vector<Detection>& detections);

/// Hungarian algorithm, rows <= cols. Returns the column of each row.
std::vector<int> solve_assignment(const Eigen::MatrixXd& cost);

struct Hit {
    const ClassConfig* cls{nullptr};
    int kf{0};
    Measurement z;
    gtsam::Point3 point;  // map frame
    std::optional<double> yaw;
    double t{0.0};
};

/// A possible new landmark.
struct Candidate {
    const ClassConfig* cls{nullptr};
    gtsam::Point3 position;
    std::vector<Hit> hits;
};

/// Unmatched detections are collected here until confirm_hits of them agree
/// within confirm_window_s.
class Candidates {
   public:
    void clear() { candidates_.clear(); }
    void add(const std::vector<Detection>& detections,
             const std::vector<std::size_t>& unmatched,
             int kf,
             const gtsam::Pose3& T_map_base,
             double t,
             const Params& params,
             const std::map<std::string, gtsam::Point3>& priors);
    std::vector<Candidate> take_confirmed(const LandmarkGraph& graph, double t);
    std::size_t size() const { return candidates_.size(); }

    /// Per class: (count, nearest distance) of detections the prior map
    /// rejected since the last call.
    std::map<std::string, std::pair<int, double>> take_prior_rejects() {
        return std::exchange(prior_rejects_, {});
    }

   private:
    std::vector<Candidate> candidates_;
    std::map<std::string, std::pair<int, double>> prior_rejects_;
};

}  // namespace vortex::landmark_server

#endif  // LANDMARK_SERVER__ASSOCIATION_HPP_
