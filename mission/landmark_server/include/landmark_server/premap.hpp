#ifndef LANDMARK_SERVER__PREMAP_HPP_
#define LANDMARK_SERVER__PREMAP_HPP_

#include <yaml-cpp/yaml.h>

#include <gtsam/geometry/Pose3.h>

#include <map>
#include <string>
#include <vector>

namespace vortex::landmark_server {

/// Where each task should be relative to the start, drawn in
/// competition_map_gui.py. Not part of the graph. File format in the README.
struct Premap {
    std::string created_at;
    std::map<std::string, gtsam::Pose3> objects;
    /// Per task, overrides the classes' prior_radius_m.
    std::map<std::string, double> radius;
    YAML::Node gui_state;

    std::map<std::string, gtsam::Point3> positions() const;
};

bool is_map_reference(const std::string& frame);

/// Bad entries are skipped and listed in `skipped`. Throws if the file cannot
/// be read or is not in the start frame.
Premap load_premap(const std::string& path, std::vector<std::string>& skipped);

std::string premap_to_yaml(const Premap& premap);

/// An existing file is first copied to <name>_<YYYYmmdd_HHMM>.yaml. Returns
/// the backup path, or "" if there was none.
std::string save_premap(const std::string& path, const Premap& premap);

std::string now_iso8601();

}  // namespace vortex::landmark_server

#endif  // LANDMARK_SERVER__PREMAP_HPP_
