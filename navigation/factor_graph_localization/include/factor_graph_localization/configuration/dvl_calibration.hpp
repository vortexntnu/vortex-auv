#ifndef FACTOR_GRAPH_LOCALIZATION__CONFIGURATION__DVL_CALIBRATION_HPP_
#define FACTOR_GRAPH_LOCALIZATION__CONFIGURATION__DVL_CALIBRATION_HPP_

#include <array>
#include <string>

#include "factor_graph_localization/factor_builders/dvl_beam.hpp"

namespace vortex::navigation {

/// Load Nucleus beams 1-3 in order using the config/sensors.yaml schema.
/// Mounting relative to STIM300 uses the vortex-utils pose schema:
/// position [m], orientation roll/pitch/yaw [deg].
/// Throws std::invalid_argument for invalid files or calibration.
std::array<DvlBeamGeometry, 3> load_dvl_beam_geometries(
    const std::string& config_path);

}  // namespace vortex::navigation

#endif  // FACTOR_GRAPH_LOCALIZATION__CONFIGURATION__DVL_CALIBRATION_HPP_
