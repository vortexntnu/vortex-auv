#include "ukf/models/strapdown_ins.hpp"
#include <vortex/utils/math.hpp>
#include "ukf/lie/so3.hpp"

StrapdownINS3D::StrapdownINS3D(const INS3DConfig& config) : config_(config) {}

State StrapdownINS3D::f(const State& state, const ImuInput& input) const {
    // TODO: strapdown integration with bias corrected imu
    return state;
}

Eigen::Matrix15d StrapdownINS3D::Q(const State& state,
                                   const ImuInput& input) const {
    // TODO: discretise G * noise_psd * G^T over input.dt
    return Eigen::Matrix15d::Zero();
}

State StrapdownINS3D::composition_plus(const State& state,
                                       const Eigen::Vector15d& delta) const {
    // TODO: right-local, q * exp(delta_theta), the rest is plain +
    return state;
}

Eigen::Vector15d StrapdownINS3D::composition_minus(const State& state_a,
                                                   const State& state_b) const {
    // TODO: log of error_quaternion(q_b, q_a), the rest is plain -
    return Eigen::Vector15d::Zero();
}
