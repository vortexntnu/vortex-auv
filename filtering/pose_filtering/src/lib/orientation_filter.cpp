#include "pose_filtering/lib/orientation_filter.hpp"
#include <algorithm>
#include <cmath>

namespace vortex::filtering {

OrientationFilter::OrientationFilter(const OrientationFilterConfig& cfg)
    : dyn_mod_(cfg.dyn_mod.std_dev),
      sensor_mod_(cfg.sensor_mod.std_dev),
      pdaf_config_(cfg.pdaf) {}

void OrientationFilter::step(
    const std::vector<Eigen::Quaterniond>& measurements,
    double dt,
    OrientationState& state) {
    if (measurements.empty()) {
        auto prediction_result =
            PDAF::predict(dyn_mod_, sensor_mod_, dt, state.error_state);
        state.error_state = prediction_result.x_pred;
        return;
    }

    Eigen::Matrix<double, 3, Eigen::Dynamic> Z(3, measurements.size());
    for (size_t i = 0; i < measurements.size(); ++i) {
        Z.col(i) = so3_log_quat(measurements.at(i) * state.q.conjugate());
    }

    auto result = PDAF::step(dyn_mod_, sensor_mod_, dt, state.error_state, Z,
                             pdaf_config_);

    Eigen::Vector3d delta = result.x_post.mean();
    state.q = so3_exp_quat(delta) * state.q;
    state.q.normalize();

    state.error_state.mean().setZero();
    state.error_state.cov() = result.x_post.cov();
}

Eigen::Vector3d OrientationFilter::so3_log_quat(
    const Eigen::Quaterniond& q_in) {
    Eigen::Quaterniond q = q_in.normalized();

    if (q.w() < 0.0) {
        q.coeffs() *= -1.0;
    }

    double norm_v = q.vec().norm();

    if (norm_v < 1e-6) {
        return 2.0 * q.vec();
    }

    double theta = 2.0 * std::atan2(norm_v, q.w());
    return theta * q.vec() / norm_v;
}

Eigen::Quaterniond OrientationFilter::so3_exp_quat(
    const Eigen::Vector3d& rvec) {
    double theta = rvec.norm();

    if (theta < 1e-6) {
        return Eigen::Quaterniond(1.0, 0.5 * rvec.x(), 0.5 * rvec.y(),
                                  0.5 * rvec.z())
            .normalized();
    }

    Eigen::Vector3d axis = rvec / theta;
    double half = 0.5 * theta;

    return Eigen::Quaterniond(std::cos(half), axis.x() * std::sin(half),
                              axis.y() * std::sin(half),
                              axis.z() * std::sin(half));
}

}  // namespace vortex::filtering
