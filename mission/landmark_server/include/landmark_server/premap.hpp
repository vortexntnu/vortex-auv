#ifndef LANDMARK_SERVER__PREMAP_HPP_
#define LANDMARK_SERVER__PREMAP_HPP_

#include <yaml-cpp/yaml.h>

#include <gtsam/geometry/Pose3.h>

#include <map>
#include <string>
#include <vector>

namespace vortex::landmark_server {

/**
 * @brief The prior map: where each task of the course should be, relative to
 * the start (the map frame), as drawn in competition_map_gui.py.
 *
 * File format (premap.yaml):
 *   reference_frame: start
 *   created_at: '2026-10-09T12:00:00'
 *   objects:
 *     torpedo: {position: [x, y, z], orientation: [qx, qy, qz, qw]}
 *   gui_state: {...}   # the GUI's drawing, kept as it is
 *
 * It never enters the graph: it only limits where a class's new landmarks
 * may appear (ClassConfig::prior, prior_radius_m) and gives search frames
 * prior_<label>.
 */
struct Premap {
    std::string created_at;
    std::map<std::string, gtsam::Pose3> objects;
    YAML::Node gui_state;

    /// Label -> position, for the candidate gate.
    std::map<std::string, gtsam::Point3> positions() const;
};

/// The reference frame names that mean the map frame (the start).
bool is_map_reference(const std::string& frame);

/**
 * @brief Read a premap file. Entries with a bad position or orientation are
 * skipped (listed in `skipped`). Throws std::runtime_error if the file
 * cannot be read or is not in the start frame.
 */
Premap load_premap(const std::string& path, std::vector<std::string>& skipped);

/// The premap as YAML text (the file format above).
std::string premap_to_yaml(const Premap& premap);

/**
 * @brief Write the premap to path. An existing file is kept as
 * <name>_<YYYYmmdd_HHMM>.yaml (its created_at, else its modification time).
 * Returns the backup path ("" if there was none). Throws on failure.
 */
std::string save_premap(const std::string& path, const Premap& premap);

/// ISO 8601 local time, seconds.
std::string now_iso8601();

}  // namespace vortex::landmark_server

#endif  // LANDMARK_SERVER__PREMAP_HPP_
