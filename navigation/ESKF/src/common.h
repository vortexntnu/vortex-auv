#pragma once
#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#include <vector>

constexpr long double PI{ 3.1415926535 };

struct Quat
{
  double w, x, y, z;
};

struct EulerAngles
{
  double roll, pitch, yaw;
};

using namespace Eigen;

MatrixXd blk3x3Diag(const Matrix3d& matrixA, const Matrix3d& matrixB, const Matrix3d& matrixC, const Matrix3d& matrixD);
Matrix3d crossProductMatrix(const Vector3d& n);
Vector4d quaternionHamiltonProduct(VectorXd quatLeft, VectorXd quatRight);
Matrix3d quaternion2Rotationmatrix(const Vector4d& quaternion);
MatrixXd jacobianFdOfDVL(const VectorXd& fun, const Quaterniond& q, const double& step, const Vector3d& velWorld);
Matrix3d eulerToRotationMatrix(const Vector3d& eulerAngles);
EulerAngles fromQuaternionToEulerAngles(const Quat& q);
Quat fromRPYToQuaternion(const EulerAngles& angles);  // yaw (Z), pitch (Y), roll (X)
double meanOfVector(const std::vector<double>& vec);
double maxOfVector(const std::vector<double>& vec);
double stanardDeviationOfVector(const std::vector<double>& vec);
