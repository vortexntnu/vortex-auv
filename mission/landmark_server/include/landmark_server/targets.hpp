#ifndef LANDMARK_SERVER__TARGETS_HPP_
#define LANDMARK_SERVER__TARGETS_HPP_

#include <string>
#include <vector>

#include "landmark_server/config.hpp"
#include "landmark_server/graph.hpp"

namespace vortex::landmark_server {

/// A TF frame to navigate by, in the map frame.
struct NamedPose {
    std::string name;
    gtsam::Pose3 pose;
};

/// The most observed landmark of the class, or nullptr.
const LandmarkState* best_of(const std::vector<LandmarkState>& landmarks,
                             const std::string& cls);

/**
 * @brief Gate frames from the two role panels, when both are mapped and
 * min_separation_m..max_separation_m apart: gate_middle between them, and
 * per panel <panel>_entrance / <panel>_exit approach_m before / after the
 * gate line and depth_below_panel_m below the panel (through its opening).
 * All have +X through the gate, away from the start side.
 */
std::vector<NamedPose> gate_frames(const std::vector<LandmarkState>& landmarks,
                                   const GateParams& gate,
                                   const gtsam::Point3& start);

/**
 * @brief Slalom frames: per row of pipes the point to pass it at, on either
 * side of its red pipe: slalom_left_<n> and slalom_right_<n>, n = 0 for the
 * row nearest the start. The mission picks the side (the same side of the
 * red pipe as the half of the gate it went through).
 *
 * Each red pipe is a row, numbered by its distance behind the first in
 * steps of row_spacing_m (a row not mapped yet leaves its number free).
 * The pass point is the middle between the red pipe
 * and the white one on that side; without that white pipe in the map, the
 * red pipe plus half of nominal_spacing_m to that side, so a row works with
 * its red pipe alone. +X goes straight through the row (across the line of
 * its pipes), away from the start; without a white pipe to give the line,
 * the heading of the row before, else the start heading (the map's x).
 * Left and right are as seen going through, away from the start.
 */
std::vector<NamedPose> slalom_frames(
    const std::vector<LandmarkState>& landmarks,
    const SlalomParams& slalom,
    const gtsam::Point3& start);

}  // namespace vortex::landmark_server

#endif  // LANDMARK_SERVER__TARGETS_HPP_
