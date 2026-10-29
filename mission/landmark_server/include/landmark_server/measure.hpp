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
    int slalom_rows{3};
    double slalom_spacing_m{2.0};
    double slalom_max_row_error_m{0.4};
};

struct Measurement {
    bool ok{false};
    Pose pose{Pose::Identity()};  // in odom
    int hits{0};
    double spread_m{0.0};
    std::string message;
    std::vector<Pose> rows;  // measure_slalom: one frame per row, nearest first
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

/// The whole slalom at once: rows slalom_spacing_m apart along one heading,
/// each row free to shift sideways. Fits that to every pipe seen and gives
/// slalom_rows frames like measure_slalom_row's, from the nearest row in front
/// of the vehicle on, also for rows not seen. pose is the first row.
Measurement measure_slalom(const std::vector<Hit>& hits,
                           const Pose& T_odom_base,
                           const MeasureSettings& s);

}  // namespace vortex::landmark_server

#endif  // LANDMARK_SERVER__MEASURE_HPP_
