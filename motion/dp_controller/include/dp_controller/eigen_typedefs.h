#ifndef VORTEX_EIGEN_TYPEDEFS_H
#define VORTEX_EIGEN_TYPEDEFS_H

#include <Eigen/Dense>

namespace Eigen {
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

typedef Eigen::Matrix<double, 7, 7> Matrix7d;
typedef Eigen::Matrix<double, 7, 1> Vector7d;

typedef Eigen::Matrix<double, 14, 1> Vector14d;

} 

#endif // VORTEX_EIGEN_TYPEDEFS_H
