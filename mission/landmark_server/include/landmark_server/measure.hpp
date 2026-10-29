#ifndef LANDMARK_SERVER__MEASURE_HPP_
#define LANDMARK_SERVER__MEASURE_HPP_

#include <string>
#include <vector>

#include "landmark_server/geometry.hpp"
#include "landmark_server/map.hpp"

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
};

/// The largest cluster of the hits. Objects without a yaw get +X from the
/// vehicle to the object.
Measurement measure_object(const std::vector<Hit>& hits,
                           const Pose& T_odom_base,
                           const MeasureSettings& s);

/// The nearest slalom row in front of the vehicle: origin at its red pipe,
/// +X through the row away from the vehicle. White pipes go to the nearest
/// red one and give the direction; with only the red pipe, +X points from
/// the vehicle to it.
Measurement measure_slalom_row(const std::vector<Hit>& hits,
                               const Pose& T_odom_base,
                               const MeasureSettings& s);

}  // namespace vortex::landmark_server

#endif  // LANDMARK_SERVER__MEASURE_HPP_
