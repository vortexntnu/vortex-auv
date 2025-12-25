#ifndef POSE_FILTERING__LIB__TYPEDEFS_HPP_
#define POSE_FILTERING__LIB__TYPEDEFS_HPP_

#include <eigen3/Eigen/Dense>
#include <vortex/utils/types.hpp>
#include <vortex_filtering/filters/ipda.hpp>
#include <vortex_filtering/vortex_filtering.hpp>

namespace vortex::filtering {

using State3d = vortex::prob::Gauss3d;
using DynMod = vortex::models::ConstantDynamicModel<3>;
using SensorMod = vortex::models::IdentitySensorModel<3, 3>;
using IPDA = vortex::filter::IPDA<DynMod, SensorMod>;
using Pose = vortex::utils::types::Pose;

struct DynModConfig {
    double std_dev{1.0};
};

struct SensorModConfig {
    double std_dev{1.0};
};

struct ExistenceManagementConfig {
    double confirmation_threshold{0.4};
    double deletion_threshold{0.2};
    double initial_existence_probability{0.3};
};

struct TrackManagerConfig {
    vortex::filter::IPDA<DynMod, SensorMod>::Config ipda;
    DynModConfig dyn_mod;
    SensorModConfig sensor_mod;
    ExistenceManagementConfig existence;
    double max_angle_gate_threshold{};
};

struct Track {
    int id{};
    State3d state;
    Eigen::Quaterniond current_orientation;
    Eigen::Quaterniond prev_orientation;
    double existence_probability{0.0};
    bool confirmed{false};

    Pose to_pose() const {
        return vortex::utils::types::Pose::from_eigen(state.mean(),
                                                      current_orientation);
    }

    double angular_distance(const Pose& z) const {
        return current_orientation.angularDistance(z.ori_quaternion());
    }
};

}  // namespace vortex::filtering

#endif  // POSE_FILTERING__LIB__TYPEDEFS_HPP_
