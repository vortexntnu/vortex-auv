#include "tukf_rsi/tukf.hpp"
#include "tukf_rsi/tukf_rsi_utils.hpp"
#include "tukf_rsi/typedefs.hpp"
#include "tukf_rsi/tukf_rsi_model.hpp"
#include <cmath>

TUKF::TUKF(const AUVState& x0,
           const Eigen::Matrix37d& Q_in,
           double dt_in)
    : x(x0), Q(Q_in), dt(dt_in), filter_failed(false), flagg(0)
{
    delta = generate_elta_matrix37() / std::sqrt(static_cast<double>(x.as_sector().size()));
    measurement_updated = MeasModel();
}

std::vector<AUVState> TUKF::sigma_points(const AUVState& current_state) {
    int n = static_cast<int>(current_state.covariance.rows());
    ++flagg;
    Eigen::Matrix37d S;
    bool chol_ok = true;
    auto llt = current_state.covariance.llt();
    if(llt.info() == Eigen::NumericalIssue) {
        chol_ok = false;
    } else {
        S = llt.matrixL();
    }
    if (!chol_ok) {
        filter_failed = true;
        S = Eigen::Matrix37d::Identity() * 1e-6;
    }
    sigma_points_list.resize(2 * n);
    for (int k = 0; k < 2 * n; ++k) {
        Eigen::Vector37d v = current_state.as_vector() + S * delta.col(k);
        sigma_points_list[k].fill_States(v);
    }
    return sigma_points_list;
}

AUVState TUKF::unscented_transform(const AUVState& current_state,
                                   const Eigen::Vector3d& control_force) {
    int n = static_cast<int>(current_state.covariance.rows());
    sigma_soints(current_state);
    y_i.resize(2 * n);
    for (int i = 0; i < 2 * n; ++i) {
        y_i[i] = F_dynamics(sigma_points_list[i], dt, control_force);
    }
    AUVState state_est;
    Eigen::Vector37d x_vec = mean_Set(y_i);
    state_est.fill_States(x_vec);
    state_est.covariance = covarianceSet(y_i, x_vec) + Q;
    return state_est;
}

void TUKF::measurement_update(const AUVState& current_state,
                              const MeasModel& measurement) {
    int n = static_cast<int>(current_state.covariance.rows());
    std::vector<MeasModel> z_i(2 * n);
    for (int i = 0; i < 2 * n; ++i) {
        z_i[i] = measurement.H(sigma_points_list[i]);
    }
    measurement_updated.measurement = mean_seasurement(z_i);
    measurement_updated.covariance = covariance_measurement(
        z_i, measurement_updated.measurement);
    cross_correlation = cross_covariance(
        y_i,
        current_state.as_vector(),
        z_i,
        measurement_updated.measurement);
}

AUVState TUKF::posterior_estimate(const AUVState& current_state,
                                  const MeasModel& measurement) {
    MeasModel nu_k;
    nu_k.measurement = measurement.measurement - measurement_updated.measurement;
    nu_k.covariance = measurement_updated.covariance + measurement.covariance;
    Eigen::Matrix<double,37,3> K = cross_correlation * nu_k.covariance.inverse();
    AUVState post;
    Eigen::Vector37d v = current_state.as_vector() +
                         K * nu_k.measurement;
    post.fill_states(v);
    post.covariance = current_state.covariance -
                      K * nu_k.covariance * K.transpose();
    return post;
}