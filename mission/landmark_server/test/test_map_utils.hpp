#ifndef LANDMARK_SERVER_TEST_MAP_UTILS_HPP_
#define LANDMARK_SERVER_TEST_MAP_UTILS_HPP_

#include <yaml-cpp/yaml.h>
#include <vortex_msgs/msg/landmark_subtype.hpp>
#include <vortex_msgs/msg/landmark_type.hpp>
#include <pose_filtering/lib/typedefs.hpp>
#include "landmark_server/class_config.hpp"

namespace vortex::mission::test {

using LT = vortex_msgs::msg::LandmarkType;
using LS = vortex_msgs::msg::LandmarkSubtype;
using vortex::filtering::LandmarkClassKey;
using vortex::filtering::Track;

/// The example configuration from config/landmark_server_config.yaml.
inline LandmarkMapConfig example_config() {
    return parse_map_config(YAML::Load(R"(
intake:
  max_pipe_distance_m: 7.0
  no_orientation_rot_variance: 1000.0
course_frame:
  gate_lock: {consistent_estimates: 10, max_yaw_std_deg: 3.0}
  warn_start_vs_gate_deg: 30.0
  lane:
    before_gate: {x: [-10.0, 45.0], y: [-12.0, 12.0]}
    after_gate: {x: [-5.0, 40.0], y: [-6.0, 6.0]}
classes:
  GATE: {max_instances: 1, retain: forever}
  SLALOM_PIPE: {max_instances: {WHITE: 10, RED: 5}, instance_gate_m: 0.7,
                retain_sec: 15.0, keep_after_observations: 30,
                min_distance_to_large_structures_m: 1.5}
  PATH_MARKER: {max_instances: 2, retain_sec: 15.0}
  TORPEDO_BOARD: {max_instances: 1, retain: forever}
  BIN: {max_instances: 4, instance_gate_m: 0.25, retain_sec: 15.0}
  TABLE: {max_instances: 1, retain: forever}
  OCTAGON: {max_instances: 1, retain: forever}
rules:
  plausibility_radius_m: 3.0
  large_structures: [GATE, TABLE, TORPEDO_BOARD, BIN_STRUCTURE]
)"));
}

inline Track make_track(int id,
                        uint16_t type,
                        uint16_t subtype,
                        const Eigen::Vector3d& position,
                        bool hit = true,
                        bool has_orientation = true,
                        const Eigen::Quaterniond& q =
                            Eigen::Quaterniond::Identity()) {
    Track t{.id = id,
            .class_key = LandmarkClassKey{type, subtype},
            .nominal_state = {position, q},
            .error_state = vortex::prob::Gauss6d(
                Eigen::Matrix<double, 6, 1>::Zero(),
                Eigen::Matrix<double, 6, 6>::Identity() * 0.01),
            .confirmed = true};
    t.hit_history.push_back(hit);
    t.has_orientation = has_orientation;
    return t;
}

}  // namespace vortex::mission::test

#endif  // LANDMARK_SERVER_TEST_MAP_UTILS_HPP_
