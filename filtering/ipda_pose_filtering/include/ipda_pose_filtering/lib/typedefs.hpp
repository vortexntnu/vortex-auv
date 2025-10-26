#ifndef TYPEDEFS_HPP
#define TYPEDEFS_HPP

#include <eigen3/Eigen/Dense>
#include <vortex_filtering/filters/ipda.hpp>
#include <vortex_filtering/vortex_filtering.hpp>

namespace Eigen {

using Vector6d = Eigen::Matrix<double, 6, 1>;

} // namespace eigen

namespace vortex::filtering {

using Measurements = Eigen::Matrix<double, 6, Eigen::Dynamic>;
using State6d = vortex::prob::Gauss6d;
using DynMod = vortex::models::ConstantPose;
using SensorMod = vortex::models::IdentityPoseSensor<6, 6>;
using IPDA = vortex::filter::IPDA<DynMod, SensorMod>;

struct DynModConfig {
    double std_pos;
    double std_orient;
};

struct SensorModConfig {
    double std_pos;
    double std_orient;
    double max_angle_gate_threshold;
};

struct ExistenceManagementConfig {
    double confirmation_threshold;
    double deletion_threshold;
    double initial_existence_probability;
};

struct TrackManagerConfig {
    vortex::filter::IPDA<DynMod, SensorMod>::Config ipda;
    DynModConfig dyn_mod;
    SensorModConfig sensor_mod;
    ExistenceManagementConfig existence;
};

struct Track {
    int id;
    State6d state;
    double existence_probability;
    bool confirmed;
};

}  // namespace vortex::filtering

#endif  // TYPEDEFS_HPP
