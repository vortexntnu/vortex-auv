#include "common.h"
#include "math.h"

Matrix3d crossProductMatrix(const Vector3d& n)
{
  Matrix3d skewMatrix = Matrix3d::Zero();

  skewMatrix << 0, -n(2), n(1), n(2), 0, -n(0), -n(1), n(0), 0;

  return skewMatrix;
}

Vector4d quaternionHamiltonProduct(VectorXd quatLeft, VectorXd quatRight)
{
  // int realPartIndex = 0;

  Vector4d quatProduct = Vector4d::Zero();
  Vector3d imaginaryPartOfLeftQuaternion = Vector3d::Zero();
  Vector3d imaginaryPartOfRightQuaternion = Vector3d::Zero();

  if (quatLeft.size() == 3)  // Assume pure quaternion
  {
    quatLeft << 0, quatLeft(0), quatLeft(1), quatLeft(2);
  }
  if (quatRight.size() == 3)  // Assume pure quaternion
  {
    quatRight << 0, quatRight(0), quatRight(1), quatRight(2);
  }

  imaginaryPartOfLeftQuaternion = quatLeft.block<3, 1>(1, 0);
  imaginaryPartOfRightQuaternion = quatRight.block<3, 1>(1, 0);

  quatProduct << quatLeft(0) * quatRight(0) -
                   imaginaryPartOfLeftQuaternion.transpose() * imaginaryPartOfRightQuaternion,
    quatRight(0) * imaginaryPartOfLeftQuaternion + quatLeft(0) * imaginaryPartOfRightQuaternion +
      crossProductMatrix(imaginaryPartOfLeftQuaternion) * imaginaryPartOfRightQuaternion;

  return quatProduct;
}

MatrixXd blk3x3Diag(const Matrix3d& matrixA, const Matrix3d& matrixB, const Matrix3d& matrixC, const Matrix3d& matrixD)
{
  int numberOfMatricies{ 4 };

  MatrixXd bdm = MatrixXd::Zero(matrixA.rows() * numberOfMatricies, matrixA.cols() * numberOfMatricies);

  for (int i = 0; i < numberOfMatricies; ++i)
  {
    if (i == 0)
    {
      bdm.block(i * matrixA.rows(), i * matrixA.cols(), matrixA.rows(), matrixA.cols()) = matrixA;
    }
    else if (i == 1)
    {
      bdm.block(i * matrixB.rows(), i * matrixB.cols(), matrixB.rows(), matrixB.cols()) = matrixB;
    }
    else if (i == 2)
    {
      bdm.block(i * matrixC.rows(), i * matrixC.cols(), matrixC.rows(), matrixC.cols()) = matrixC;
    }
    else if (i == 3)
    {
      bdm.block(i * matrixD.rows(), i * matrixD.cols(), matrixD.rows(), matrixD.cols()) = matrixD;
    }
    else
    {
      std::cout << "ERROR: Could not make a blkdiag matrix" << std::endl;
    }

    // bdm.block(i * a.rows(), i * a.cols(), a.rows(), a.cols()) = a;
  }

  return bdm;
}

Matrix3d quaternion2Rotationmatrix(const Vector4d& quaternion)
{
  Matrix3d RotationMatrix = Matrix3d::Zero();
  Matrix3d S = Matrix3d::Zero();
  Vector3d imaginaryPart = Vector3d::Zero();
  double realPart{ 0 };

  realPart = quaternion(0);
  imaginaryPart = quaternion.segment(1, quaternion.size() - 1);

  S = crossProductMatrix(imaginaryPart);
  RotationMatrix = MatrixXd::Identity(3, 3) + (2 * realPart * S) + (2 * S * S);

  return RotationMatrix;
}

MatrixXd jacobianFdOfDVL(const VectorXd& fun, const Quaterniond& q, const double& step, const Vector3d& velWorld)
{
  Vector4d x{ q.w(), q.x(), q.y(), q.z() };
  Vector4d xPerturbed = Vector4d::Zero();
  Vector3d fPerturbed = Vector3d::Zero();
  int numberOfRowsOfQuaternion{ 4 };
  int numberOfRowsOfF{ 0 };

  numberOfRowsOfF = fun.rows();

  MatrixXd jacobianMatrix(numberOfRowsOfF, numberOfRowsOfQuaternion);

  for (int i = 0; i < numberOfRowsOfQuaternion; ++i)
  {
    xPerturbed = x;
    xPerturbed(i) = xPerturbed(i) + step;
    fPerturbed = quaternion2Rotationmatrix(xPerturbed) * velWorld;
    jacobianMatrix.col(i) = (fPerturbed - fun) / step;  // Check if error
  }

  return jacobianMatrix;
}

Matrix3d eulerToRotationMatrix(const Vector3d& eulerAngles)
{
  Matrix3d rotation_matrix = Matrix3d::Zero();
  Matrix3d rotation_matrix_roll = Matrix3d::Zero();
  Matrix3d rotation_matrix_pitch = Matrix3d::Zero();
  Matrix3d rotation_matrix_yaw = Matrix3d::Zero();

  double roll{ eulerAngles(0) };
  double pitch{ eulerAngles(1) };
  double yaw{ eulerAngles(2) };

  rotation_matrix_roll << 1, 0, 0, 0, cos(roll), -1.0 * sin(roll), 0, sin(roll), cos(roll);

  rotation_matrix_pitch << cos(pitch), 0, sin(pitch), 0, 1, 0, -1.0 * sin(pitch), 0, cos(pitch);

  rotation_matrix_yaw << cos(yaw), -1.0 * sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;

  rotation_matrix = rotation_matrix_yaw * rotation_matrix_pitch * rotation_matrix_roll;

  return rotation_matrix;
}

EulerAngles fromQuaternionToEulerAngles(const Quat& q)
{
  EulerAngles angles;

  // roll (x-axis rotation)
  double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
  double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
  angles.roll = std::atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = 2 * (q.w * q.y - q.z * q.x);
  if (std::abs(sinp) >= 1)
    angles.pitch = std::copysign(M_PI / 2, sinp);  // use 90 degrees if out of range
  else
    angles.pitch = std::asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  angles.yaw = std::atan2(siny_cosp, cosy_cosp);

  return angles;
}

Quat fromRPYToQuaternion(const EulerAngles& angles)  // yaw (Z), pitch (Y), roll (X)
{
  // Abbreviations for the various angular functions
  double cy = cos(angles.yaw * 0.5);
  double sy = sin(angles.yaw * 0.5);
  double cp = cos(angles.pitch * 0.5);
  double sp = sin(angles.pitch * 0.5);
  double cr = cos(angles.roll * 0.5);
  double sr = sin(angles.roll * 0.5);

  Quat q;
  q.w = cy * cp * cr + sy * sp * sr;
  q.x = cy * cp * sr - sy * sp * cr;
  q.y = sy * cp * sr + cy * sp * cr;
  q.z = sy * cp * cr - cy * sp * sr;

  return q;
}

double meanOfVector(const std::vector<double>& vec)
{
  double sum = 0;

  for (auto& each : vec)
    sum += each;

  return sum / vec.size();
}

double maxOfVector(const std::vector<double>& vec)
{
  double max = *std::max_element(vec.begin(), vec.end());

  return max;
}

double stanardDeviationOfVector(const std::vector<double>& vec)
{
  double square_sum_of_difference = 0;
  double mean_var = meanOfVector(vec);
  auto len = vec.size();

  double tmp;
  for (auto& each : vec)
  {
    tmp = each - mean_var;
    square_sum_of_difference += tmp * tmp;
  }

  return std::sqrt(square_sum_of_difference / (len - 1));
}