/**
 * @file
 * @brief An extension of the eigen library containing useful
 * functionality for matrix computation
*/


#ifndef VORTEX_EIGEN_HELPER_H
#define VORTEX_EIGEN_HELPER_H

#include "vortex_msgs/ThrusterForces.h"
#include "ros/ros.h"
#include <Eigen/Dense>

#include <string>
#include <vector>

/**
 * @brief check if a matrix is valid (no nan or inf entries)
 * 
 * @param X   The matrix to be checked
 * 
 * @return true if X has any nan or inf elements, false otherwise
*/
template<typename Derived>
inline bool isFucked(const Eigen::MatrixBase<Derived>& X)
{
  bool has_nan = !(X.array() == X.array()).all();
  bool has_inf = !((X - X).array() == (X - X).array()).all();
  return has_nan || has_inf;
}


/**
 * @brief read a matrix from the ROS parameter server
 * 
 * @param nh    ROS NodeHandle
 * @param name  Parameter name of the matrix in the parameter server
 * @param X     The matrix that will be filled with the values read from the parameter server
 * 
 * @return false if unsuccessful, true if successful
*/
inline bool getMatrixParam(ros::NodeHandle nh, std::string name, Eigen::MatrixXd *X)
{
  XmlRpc::XmlRpcValue matrix;
  nh.getParam(name, matrix);

  try
  {
    const int rows = matrix.size();
    const int cols = matrix[0].size();
    X->setZero(rows, cols);
    for (int i = 0; i < rows; ++i)
      for (int j = 0; j < cols; ++j)
        (*X)(i, j) = matrix[i][j];
  }
  catch(...)
  {
    return false;
  }
  return true;
}

/**
 * @brief print Eigen matrix
 * 
 * @param name    Name of matrix
 * @param X       Matrix
*/
inline void printEigen(std::string name, const Eigen::MatrixXd &X)
{
  std::stringstream ss;
  ss << std::endl << name << " = " << std::endl << X;
  ROS_INFO_STREAM(ss.str());
}


/**
 * @brief calculate pseudoinverse matrix
 * 
 * @param X         The matrix to be inverted
 * @param X_pinv    The inverted version of the @p X matrix
 * 
 * @return false if the calculation fails, true otherwise
*/
inline bool pseudoinverse(const Eigen::MatrixXd &X, Eigen::MatrixXd *X_pinv)
{
  Eigen::MatrixXd copy = X.transpose() * (X * X.transpose()).inverse();

  if (isFucked(copy))
  {
    return false;
  }
  *X_pinv = copy;
  return true;
}


/**
 * @brief calculate a 3x3 skew-symmetric matrix
 * 
 * @param v   The vector from which the matrix is to be calculated
 * 
 * @return the 3x3 skew-symmetric matrix of @p v
*/
inline Eigen::Matrix3d skew(const Eigen::Vector3d &v)
{
  Eigen::Matrix3d S;
  S <<  0   , -v(2),  v(1),
        v(2),  0   , -v(0),
       -v(1),  v(0),  0;
  return S;
}


/**
 * @brief Convert a vector to a ThrusterForces vortex message
 * 
 * @param u     The Eigen::vector to be converted
 * @param msg   The vortex_msgs output message  
 * 
*/
inline void arrayEigenToMsg(const Eigen::VectorXd &u, vortex_msgs::ThrusterForces *msg)
{
  int r = u.size();
  std::vector<double> u_vec(r);
  for (int i = 0; i < r; ++i)
    u_vec[i] = u(i);
  msg->thrust = u_vec;
}


/**
 * @brief saturate a vector to a given range
 * 
 * @param v     The vector to be saturated
 * @param min   Lower saturation limit
 * @param max   Upper saturation limit
 * 
 * @return true if all elements are in the [ @p min @p max ] range already
 * false otherwise
 * 
*/
inline bool saturateVector(Eigen::VectorXd *v, double min, double max)
{
  bool vector_in_range = true;
  for (int i = 0; i < v->size(); ++i)
  {
    if ((*v)(i) > max)
    {
      (*v)(i) = max;
      vector_in_range = false;
    }
    else if ((*v)(i) < min)
    {
      (*v)(i) = min;
      vector_in_range = false;
    }
  }
  return vector_in_range;
}

#endif  // VORTEX_EIGEN_HELPER_H
