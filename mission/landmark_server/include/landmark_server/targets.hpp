#ifndef LANDMARK_SERVER__TARGETS_HPP_
#define LANDMARK_SERVER__TARGETS_HPP_

#include <string>
#include <vector>

#include "landmark_server/config.hpp"
#include "landmark_server/geometry.hpp"
#include "landmark_server/map.hpp"

namespace vortex::landmark_server {

/// A TF frame in odom.
struct NamedPose {
    std::string name;
    Pose pose;
};

/// The most observed landmark of the class, or nullptr.
const Landmark* best_of(const std::vector<Landmark>& landmarks,
                        const std::string& cls);

/// gate_middle and <panel>_entrance / <panel>_exit, +X through the gate away
/// from start (a point on the near side of the gate). With one panel mapped:
/// only that panel's two frames, +X along the line from start to the panel.
std::vector<NamedPose> gate_frames(const std::vector<Landmark>& landmarks,
                                   const GateParams& gate,
                                   const Eigen::Vector3d& start);

}  // namespace vortex::landmark_server

#endif  // LANDMARK_SERVER__TARGETS_HPP_
