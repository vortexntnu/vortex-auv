/**
 * @file eigen_typedefs.hpp
 * @brief Contains the typedef for a 6x1 Eigen vector and a 6x6 Eigen matrix.
 */

#ifndef PID_CONTROLLER_DP__TYPEDEFS_HPP_
#define PID_CONTROLLER_DP__TYPEDEFS_HPP_

#include <eigen3/Eigen/Dense>
#include <vortex/utils/types.hpp>

namespace types {
using Vector3d = Eigen::Matrix<double, 3, 1>;
using Vector4d = Eigen::Matrix<double, 4, 1>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Matrix3d = Eigen::Matrix<double, 3, 3>;
using Matrix4x3d = Eigen::Matrix<double, 4, 3>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix7x6d = Eigen::Matrix<double, 7, 6>;
using Matrix6x7d = Eigen::Matrix<double, 6, 7>;
using Matrix7d = Eigen::Matrix<double, 7, 7>;
using Quaterniond = Eigen::Quaterniond;

// Alias canonical types from vortex utils
using Eta = ::vortex::utils::types::Pose;
using Nu = ::vortex::utils::types::Twist;

struct J_transformation {
    Matrix3d R = Matrix3d::Identity();
    Matrix4x3d T = Matrix4x3d::Zero();

    Matrix7x6d as_matrix() const {
        Matrix7x6d mat = Matrix7x6d::Zero();
        mat.block<3, 3>(0, 0) = R;
        mat.block<4, 3>(3, 3) = T;
        return mat;
    }
};
}  // namespace types

#endif  // PID_CONTROLLER_DP__TYPEDEFS_HPP_
