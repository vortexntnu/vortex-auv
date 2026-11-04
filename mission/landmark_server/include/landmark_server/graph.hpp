#ifndef LANDMARK_SERVER__GRAPH_HPP_
#define LANDMARK_SERVER__GRAPH_HPP_

#include <Eigen/Core>

#include <memory>
#include <optional>
#include <vector>

#include "landmark_server/config.hpp"
#include "landmark_server/geometry.hpp"
#include "landmark_server/landmark.hpp"

namespace vortex::landmark_server {

/// iSAM2 graph of keyframes and landmark points in map. Its only output for
/// the rest of the stack is map -> odom. GTSAM stays out of the other files.
class DriftGraph {
   public:
    DriftGraph();
    ~DriftGraph();

    void reset(const GraphParams& params,
               const Pose& T_map_start,
               const Pose& T_odom_base,
               double t);
    /// Keeps the detections, and adds a keyframe with them when it is time.
    void update(const Pose& T_odom_base,
                double t,
                const std::vector<Observation>& observations);
    Pose T_map_odom() const;
    /// The graph's estimate, once the landmark has a keyframe. Not for
    /// navigation.
    std::optional<Eigen::Vector3d> landmark(int id) const;
    /// Squared Mahalanobis distance of each detection (in odom, rows) to each
    /// landmark (cols), with the uncertainty of the pose, the landmark and the
    /// detection. Infinite for a landmark not in the graph yet.
    Eigen::MatrixXd squared_distances(
        const std::vector<Eigen::Vector3d>& detections,
        const std::vector<int>& ids) const;
    /// Squared Mahalanobis distance between two landmarks in the graph.
    double squared_distance_between(int a, int b) const;

   private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace vortex::landmark_server

#endif  // LANDMARK_SERVER__GRAPH_HPP_
