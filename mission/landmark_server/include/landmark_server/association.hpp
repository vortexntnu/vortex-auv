#ifndef LANDMARK_SERVER__ASSOCIATION_HPP_
#define LANDMARK_SERVER__ASSOCIATION_HPP_

#include <Eigen/Core>

#include <deque>
#include <map>
#include <string>
#include <vector>

#include "landmark_server/config.hpp"
#include "landmark_server/graph.hpp"
#include "landmark_server/landmark.hpp"

namespace vortex::landmark_server {

/// Lowest total cost, one column per row. cost needs rows <= cols.
std::vector<int> solve_assignment(const Eigen::MatrixXd& cost);

/// Which object is this detection? Hungarian on the Mahalanobis distances from
/// the graph's last solve. Objects stay in odom, as the median of their newest
/// detections, so the graph never moves what the vehicle drives on.
class Associator {
   public:
    void reset(const Config& cfg);
    /// Returns the detections that belong to an object.
    std::vector<Observation> update(const std::vector<Hit>& hits,
                                    double t,
                                    const DriftGraph& graph);
    /// Forgets the landmarks of one class until seen again, "" for all.
    void clear(const std::string& class_name = "");
    const std::vector<Landmark>& landmarks() const { return landmarks_; }

   private:
    struct Entry {
        std::string group;
        std::deque<Hit> recent;
        std::map<const ClassConfig*, int> votes;
        int n_obs{0};
        double first_seen{0.0};
    };
    struct Candidate {
        std::string group;
        std::vector<Hit> hits;
        Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    };

    void record(int id, const Hit& hit, std::vector<Observation>& out);
    void add_candidate(const Hit& hit);
    void confirm_candidates(double t,
                            const DriftGraph& graph,
                            std::vector<Observation>& out);
    void retire_duplicates(const DriftGraph& graph);
    void rebuild_landmarks();

    const Config* cfg_{nullptr};
    double gate_d2_{0.0};
    int next_id_{1};
    std::map<int, Entry> entries_;
    std::vector<Candidate> candidates_;
    std::vector<Landmark> landmarks_;
};

}  // namespace vortex::landmark_server

#endif  // LANDMARK_SERVER__ASSOCIATION_HPP_
