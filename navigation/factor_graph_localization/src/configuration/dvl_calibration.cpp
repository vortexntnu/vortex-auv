#include "factor_graph_localization/configuration/dvl_calibration.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <stdexcept>
#include <vortex/utils/waypoint_utils.hpp>

namespace vortex::navigation {
namespace {

gtsam::Vector3 read_vector3(const YAML::Node& node, const std::string& name) {
    if (!node.IsSequence() || node.size() != 3) {
        throw std::invalid_argument(name +
                                    " must contain three finite numbers");
    }

    try {
        const gtsam::Vector3 value{node[0].as<double>(), node[1].as<double>(),
                                   node[2].as<double>()};
        if (!value.allFinite()) {
            throw std::invalid_argument(name +
                                        " must contain three finite numbers");
        }
        return value;
    } catch (const YAML::Exception&) {
        throw std::invalid_argument(name +
                                    " must contain three finite numbers");
    }
}

gtsam::Unit3 read_beam_direction(const YAML::Node& dvl,
                                 const std::string& field) {
    const std::string name = "dvl." + field;
    const gtsam::Vector3 beam_direction = read_vector3(dvl[field], name);
    const double norm = beam_direction.stableNorm();
    if (!std::isfinite(norm) || norm <= 0.0) {
        throw std::invalid_argument(name + " must have a finite, nonzero norm");
    }
    return gtsam::Unit3(beam_direction / norm);
}

}  // namespace

std::array<DvlBeamGeometry, 3> load_dvl_beam_geometries(
    const std::string& config_path) {
    try {
        const YAML::Node root{YAML::LoadFile(config_path)};
        const auto& dvl = root["dvl"];

        const auto mounting =
            vortex::utils::waypoints::load_pose_from_yaml(config_path, "dvl");
        if (!mounting.to_vector().allFinite()) {
            throw std::invalid_argument(
                "dvl pose must contain finite position and orientation values");
        }
        const gtsam::Pose3 IMU_P_DVL{gtsam::Rot3{mounting.ori_quaternion()},
                                     mounting.pos_vector()};

        return {
            {{read_beam_direction(dvl, "beam_1_direction_dvl"), IMU_P_DVL},
             {read_beam_direction(dvl, "beam_2_direction_dvl"), IMU_P_DVL},
             {read_beam_direction(dvl, "beam_3_direction_dvl"), IMU_P_DVL}}};
    } catch (const std::runtime_error& error) {
        throw std::invalid_argument("Invalid calibration file '" + config_path +
                                    "': " + error.what());
    } catch (const std::invalid_argument& error) {
        throw std::invalid_argument("Invalid calibration file '" + config_path +
                                    "': " + error.what());
    }
}

}  // namespace vortex::navigation
