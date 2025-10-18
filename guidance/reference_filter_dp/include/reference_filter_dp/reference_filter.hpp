#ifndef REFERENCE_FILTER_HPP
#define REFERENCE_FILTER_HPP

#include "reference_filter_dp/eigen_typedefs.hpp"

namespace vortex::guidance {

struct ReferenceFilterParams {
    Eigen::Vector6d omega = Eigen::Vector6d::Zero();
    Eigen::Vector6d zeta = Eigen::Vector6d::Zero();
};

class ReferenceFilter {
   public:
    explicit ReferenceFilter(const ReferenceFilterParams& params);

    // @brief Calculate the state derivative
    // @param x The state vector 18x1
    // @param r The reference vector 6x1
    // @return The state derivative 18x1
    // REF: Handbook of Marine Craft Hydrodynamics and Motion Control, Fossen
    // 2021 p. 337 eq: 12.11
    Eigen::Vector18d calculate_x_dot(const Eigen::Vector18d& x,
                                     const Eigen::Vector6d& r);

    // @brief Calculate the state transition matrix
    // REF: Handbook of Marine Craft Hydrodynamics and Motion Control, Fossen
    // 2021 p. 337 eq: 12.12
    void calculate_Ad(const Eigen::Vector6d& omega,
                      const Eigen::Vector6d& zeta);

    // @brief Calculate the input matrix
    // REF: Handbook of Marine Craft Hydrodynamics and Motion Control, Fossen
    // 2021 p. 337 eq: 12.12
    void calculate_Bd(const Eigen::Vector6d& omega);

   private:
    Eigen::Matrix18d Ad_ = Eigen::Matrix18d::Zero();
    Eigen::Matrix18x6d Bd_ = Eigen::Matrix18x6d::Zero();
};

}  // namespace vortex::guidance

#endif  // REFERENCE_FILTER_HPP
