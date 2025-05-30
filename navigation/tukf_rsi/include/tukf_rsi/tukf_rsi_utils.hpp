#ifndef TUKF_RSI_UTILS_HPP
#define TUKF_RSI_UTILS_HPP

#include <vector>
#include <eigen3/Eigen/Dense>
#include "tukf_rsi/typedefs.hpp"

// @brief Compute mean quaternion from a set of quaternions
// @param quats: Vector of quaternions
// @param tol: Tolerance for convergence
// @param maxIter: Maximum number of iterations
Eigen::Quaterniond quaternion_mean(
    const std::vector<Eigen::Quaterniond>& quats,
    double tol = 1e-6,
    int maxIter = 100
);

// @brief Compute mean of a set of AUV states
// @param setPoints: Vector of AUVState objects
// @param tol: Tolerance for convergence
// @param maxIter: Maximum number of iterations
Eigen::Vector37d mean_set(
    const std::vector<AUVState>& setPoints,
    double tol = 1e-6,
    int maxIter = 100
);

// @brief Compute mean of a set of measurements
// @param setPoints: Vector of MeasModel objects
Eigen::Vector3d mean_masurement(const std::vector<MeasModel>& setPoints);

// @brief Compute covariance of a set of AUV states
// @param setPoints: Vector of AUVState objects
// @param meanVec: Mean vector of the set
// @param tol: Tolerance for convergence
Eigen::Matrix37d covariance_set(
    const std::vector<AUVState>& setPoints,
    const Eigen::Vector37d& meanVec,
    double tol = 1e-6
);

// @brief Compute covariance of a set of measurements
// @param setPoints: Vector of MeasModel objects
// @param mean: Mean vector of the measurements
Eigen::Matrix3d covariance_measurement(
    const std::vector<MeasModel>& setPoints,
    const Eigen::Vector3d& mean
);

// @brief Compute cross-covariance between AUV states and measurements
// @param setY: Vector of AUVState objects
// @param meanY: Mean vector of the AUV states
// @param setZ: Vector of MeasModel objects
// @param meanZ: Mean vector of the measurements
// @param tol: Tolerance for convergence
Eigen::Matrix<double,37,3> cross_covariance(
    const std::vector<AUVState>& setY,
    const Eigen::Vector37d& meanY,
    const std::vector<MeasModel>& setZ,
    const Eigen::Vector3d& meanZ,
    double tol = 1e-6
);

#endif // TUKF_RSI_UTILS_HPP

