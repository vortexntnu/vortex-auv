#ifndef LANDMARK_SERVER__MEASURE_HPP_
#define LANDMARK_SERVER__MEASURE_HPP_

#include <string>
#include <utility>
#include <vector>

#include "landmark_server/geometry.hpp"
#include "landmark_server/landmark.hpp"

namespace vortex::landmark_server {

struct MeasureSettings {
    double cluster_radius_m{0.5};
    int min_hits{5};
    double max_spread_m{0.3};
};

struct Measurement {
    bool ok{false};
    Pose pose{Pose::Identity()};  // in odom
    int hits{0};
    double spread_m{0.0};
    std::string message;
    // Frames on the measured one: <out_frame>/<name>, in odom.
    std::vector<std::pair<std::string, Pose>> children;
};

/// The largest cluster of the hits. Objects without a yaw get +X from the
/// vehicle to the object.
Measurement measure_object(const std::vector<Hit>& hits,
                           const Pose& T_odom_base,
                           const MeasureSettings& s);

}  // namespace vortex::landmark_server

#endif  // LANDMARK_SERVER__MEASURE_HPP_
