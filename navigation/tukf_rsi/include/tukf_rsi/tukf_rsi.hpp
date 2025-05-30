// tukf.hpp
#ifndef TUKF_HPP
#define TUKF_HPP

#include "typedefs.hpp" // includes AUVState, MeasModel, and utility functions
#include <vector>

class TUKF {
public:
    TUKF(const AUVState& x0,
         const Eigen::Matrix37d& Q_in,
         double dt = 0.01);

    // @brief Generate sigma points for the current state
    // @param current_state: Current state of the AUV
    // @return A vector of sigma points
    std::vector<AUVState> sigma_points(const AUVState& current_state);

    // @brief Unscented transform to predict the next state
    // @param current_state: Current state of the AUV
    // @param control_force: Control force applied to the AUV
    // @return Predicted state after applying the control force
    AUVState unscented_transform(const AUVState& current_state,
                                const Eigen::Vector3d& control_force);

    // @brief Perform measurement update using the measurement model
    // @param current_state: Current state of the AUV
    // @param measurement: Measurement model containing the measurement and covariance
    // @return Updated state after measurement update
    void measurement_update(const AUVState& current_state,
                           const MeasModel& measurement);

    // @brief Posterior estimate of the state after measurement update
    // @param current_state: Current state of the AUV
    // @param measurement: Measurement model containing the measurement and covariance
    // @return Posterior estimate of the state
    AUVState posterior_estimate(const AUVState& current_state,
                               const MeasModel& measurement);

    // public state and flags
    AUVState x;
    bool filter_failed;

private:

    Eigen::Matrix37d Q;

    Eigen::Matrix<double,37,74> delta;

    std::vector<AUVState> sigma_points_list;

    std::vector<AUVState> y_i;

    MeasModel measurement_updated;

    Eigen::Matrix<double,37,3> cross_correlation;

    double dt;
    
    int flagg;
};

#endif // TUKF_HPP