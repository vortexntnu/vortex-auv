#ifndef LANDMARK_SLAM__TARGETS_HPP_
#define LANDMARK_SLAM__TARGETS_HPP_

#include <gtsam/geometry/Pose3.h>

#include <string>
#include <vector>

#include "landmark_slam/graph.hpp"

namespace vortex::landmark_slam {

/// A task frame derived from landmarks, in the map frame.
struct TargetFrame {
    std::string name;
    gtsam::Pose3 pose;
};

/**
 * @brief Gate frames from its two role panels, when both are mapped and
 * between min_separation_m and max_separation_m apart. All have +X
 * through the gate, away from the start side, and +Z down:
 * gate_middle between the panels, and per panel <panel>_entrance and
 * <panel>_exit, approach_m before and after the panel and
 * depth_below_panel_m below it: through that role's opening.
 */
std::vector<TargetFrame> gate_frames(
    const std::vector<LandmarkState>& landmarks,
    const GateParams& gate,
    const gtsam::Point3& start);

}  // namespace vortex::landmark_slam

#endif  // LANDMARK_SLAM__TARGETS_HPP_
