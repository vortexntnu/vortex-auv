#include "reference_filter_dp/reference_filter.hpp"
#include <spdlog/spdlog.h>

namespace vortex::guidance {

ReferenceFilter::ReferenceFilter(const ReferenceFilterParams& params) {
    calculate_Ad(params.omega, params.zeta);
    calculate_Bd(params.omega);
}

Eigen::Vector18d ReferenceFilter::calculate_x_dot(const Eigen::Vector18d& x,
                                                  const Eigen::Vector6d& r) {
    Eigen::Vector18d x_dot = Ad_ * x + Bd_ * r;

    return x_dot;
}

void ReferenceFilter::calculate_Ad(const Eigen::Vector6d& omega,
                                   const Eigen::Vector6d& zeta) {
    Eigen::Matrix6d omega_diag = omega.asDiagonal();
    Eigen::Matrix6d delta = zeta.asDiagonal();
    Eigen::Matrix6d omega_diag_squared = omega_diag * omega_diag;
    Eigen::Matrix6d omega_diag_cubed = omega_diag * omega_diag * omega_diag;
    Ad_.block<6, 6>(0, 6) = Eigen::Matrix6d::Identity();
    Ad_.block<6, 6>(12, 0) = -omega_diag_cubed;
    Ad_.block<6, 6>(12, 6) =
        -(2 * delta + Eigen::Matrix6d::Identity()) * omega_diag_squared;
    Ad_.block<6, 6>(12, 12) =
        -(2 * delta + Eigen::Matrix6d::Identity()) * omega_diag;
    Ad_.block<6, 6>(6, 12) = Eigen::Matrix6d::Identity();
}

void ReferenceFilter::calculate_Bd(const Eigen::Vector6d& omega) {
    Eigen::Matrix6d omega_diag = omega.asDiagonal();
    Bd_.block<6, 6>(12, 0) = omega_diag * omega_diag * omega_diag;
}

}  // namespace vortex::guidance
