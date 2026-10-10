#ifndef LANDMARK_SERVER__TARGETS_HPP_
#define LANDMARK_SERVER__TARGETS_HPP_

#include <string>
#include <vector>

#include "landmark_server/config.hpp"
#include "landmark_server/graph.hpp"

namespace vortex::landmark_server {

/// A TF frame in the map frame.
struct NamedPose {
    std::string name;
    gtsam::Pose3 pose;
};

/// The most observed landmark of the class, or nullptr.
const LandmarkState* best_of(const std::vector<LandmarkState>& landmarks,
                             const std::string& cls);

/// gate_middle and <panel>_entrance / <panel>_exit, +X through the gate away
/// from the start. Empty until both panels are mapped.
std::vector<NamedPose> gate_frames(const std::vector<LandmarkState>& landmarks,
                                   const GateParams& gate,
                                   const gtsam::Point3& start);

/// slalom_left_<n> and slalom_right_<n>: where to pass row n on each side
/// of its red pipe, +X through the row. Row 0 is nearest the start.
std::vector<NamedPose> slalom_frames(
    const std::vector<LandmarkState>& landmarks,
    const SlalomParams& slalom,
    const gtsam::Point3& start);

/// torpedo_opening_<name> per configured opening, +X through the board.
std::vector<NamedPose> torpedo_frames(
    const std::vector<LandmarkState>& landmarks,
    const TorpedoParams& torpedo);

}  // namespace vortex::landmark_server

#endif  // LANDMARK_SERVER__TARGETS_HPP_
