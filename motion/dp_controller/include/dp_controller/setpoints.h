/*   Written by Kristoffer Rakstad Solberg, Student
     Documented by Christopher Strøm
     Copyright (c) 2019 Manta AUV, Vortex NTNU.
     All rights reserved. */

/**
 * @file
 * @brief Class declaration for the Setpoint class
*/


#ifndef VORTEX_CONTROLLER_SETPOINTS_H
#define VORTEX_CONTROLLER_SETPOINTS_H

#include <Eigen/Dense>
#include "eigen_typedefs.h"


/**
 * @brief a class containing the current setpoint data for wrench, position and orientation
*/
class Setpoints
{
public:
  /**
   * @brief Setpoints class constructor
   * 
  */
  Setpoints();


  /**
   * @brief Get wrench setpoint
   * 
   * @param wrench    A pointer whos values will be set to zeros
   * 
   * @return false if current wrench setpoint is invalid, true otherwise
  */
  bool getZero(Eigen::Vector6d *wrench);


  /**
   * @brief Get position and orientation setpoints
   * 
   * @param position        A pointer whos value will be set to the current position setpoint
   * @param orientation     A pointer whos value will be set to the current orientation setpoint
   * 
   * @return false if current pose is invalid, true otherwise
  */
  bool get(Eigen::Vector3d    *position,
           Eigen::Quaterniond *orientation);


  /**
   * @brief Get position setpoint
   * 
   * @param position        A pointer whos value will be set to the current position setpoint
   * 
   * @return false if current pose is invalid, true otherwise
  */
  bool get(Eigen::Vector3d *position);


  /**
   * @brief Get orientation setpoint
   * 
   * @param orientation     A pointer whos value will be set to the current orientation setpoint
   * 
   * @return false if current pose is invalid, true otherwise
  */
  bool get(Eigen::Quaterniond *orientation);


  /**
   * @brief Get orientation setpoint in euler angles
   * 
   * @param orientation     A pointer whos value will be set to the current orientation setpoint
   * 
   * @return false if current pose is invalid, true otherwise
  */
  bool getEuler(Eigen::Vector3d *orientation);


  /**
   * @brief Set position and orientation setpoints
   * 
   * @param position      The new position setpoint, expressed as a 3d vector
   * @param orientation   The new orientation setpoint, expressed as a quaternion  
  */
  void set(const Eigen::Vector3d    &position,
           const Eigen::Quaterniond &orientation);


  /**
   * @brief Set position setpoint
   * 
   * @param position      The new position setpoint, expressed as a 3d vector
  */
  void set(const Eigen::Vector3d    &position);


  /**
   * @brief Set orientation setpoint
   * 
   * @param orientation   The new orientation setpoint, expressed as a quaternion  
   *
  */
  void set(const Eigen::Quaterniond &orientation);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
  Eigen::Vector6d    m_wrench;        /** 6d vector containing wrench setpoint values   */
  Eigen::Vector3d    m_position;      /** 6d vector containing position setpoint values */
  Eigen::Quaterniond m_orientation;   /** Quaternion containing orientation setpoints   */

  bool   m_wrench_is_valid;           /** Determines if current wrench values are valid */
  bool   m_pose_is_valid;             /** Determines if current pose is valid           */
};

#endif  // VORTEX_CONTROLLER_SETPOINTS_H
