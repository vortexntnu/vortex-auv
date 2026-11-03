#include <cmath>

template <SensorModelConcept Sensor>
bool ESKF::measurement_update(const Sensor& measurement,
                              double threshold,
                              double& nis) {
    constexpr int N = Sensor::dimension;
    using MeasurementMatrix = Eigen::Matrix<double, N, N>;
    const auto innovation = measurement.innovation(current_nom_state_);
    const auto H = measurement.jacobian(current_nom_state_);
    const auto R = measurement.noise_covariance();
    nis = std::numeric_limits<double>::quiet_NaN();
    if (!innovation.allFinite() || !H.allFinite() || !R.allFinite() ||
        !R.isApprox(R.transpose(), 1e-10) ||
        Eigen::LLT<MeasurementMatrix>(R).info() != Eigen::Success)
        return false;
    const Eigen::Matrix15d P = current_error_state_.covariance;
    const MeasurementMatrix S = H * P * H.transpose() + R;
    const Eigen::LLT<MeasurementMatrix> solver(S);
    if (solver.info() != Eigen::Success)
        return false;
    nis = innovation.dot(solver.solve(innovation));
    if (!std::isfinite(nis) || nis < 0 || nis > threshold)
        return false;
    const Eigen::Matrix<double, 15, N> K =
        solver.solve((P * H.transpose()).transpose()).transpose();
    const Eigen::Vector15d correction = K * innovation;
    const Eigen::Matrix15d residual = Eigen::Matrix15d::Identity() - K * H;
    const Eigen::Matrix15d posterior =
        residual * P * residual.transpose() + K * R * K.transpose();
    if (!correction.allFinite() || !posterior.allFinite())
        return false;
    const auto before_nominal = current_nom_state_;
    const auto before_error = current_error_state_;
    current_error_state_.set_from_vector(correction);
    current_error_state_.covariance = posterior;
    injection_and_reset();
    if (!current_nom_state_.as_vector().allFinite() ||
        !current_error_state_.covariance.allFinite()) {
        current_nom_state_ = before_nominal;
        current_error_state_ = before_error;
        return false;
    }
    return true;
}
