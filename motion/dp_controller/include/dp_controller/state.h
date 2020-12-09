/*   Written by Kristoffer Rakstad Solberg, Student
     Documented by Christopher Strøm
     Copyright (c) 2019 Manta AUV, Vortex NTNU.
     All rights reserved. */

/**
 * @file
 * @brief Class declaration for the Setpoint class
*/


#ifndef VORTEX_CONTROLLER_STATE_H
#define VORTEX_CONTROLLER_STATE_H

#include <Eigen/Dense>
#include "eigen_typedefs.h"

using namespace Eigen;

/**
 * @brief a class containing the current state data for position, orientation and velocity
*/
class State
{
public:

  /**
   * @brief State class constructor
   * 
   * States for position and velocity are all set to zero,
   * while orientation is initialized as the 6x6 identity matrix.
  */
  State();


  /**
   * @brief Get position and orientation states
   * 
   * @param position        A pointer whos value will be set to the current position state
   * @param orientation     A pointer whos value will be set to the current orientation state
   * 
   * @return false if states are not initialized, true otherwise
  */
  bool get(Eigen::Vector3d      *position,
           Eigen::Quaterniond   *orientation);


  /**
   * @brief Get position, orientation and velocity states
   * 
   * @param position        A pointer whos value will be set to the current position state
   * @param orientation     A pointer whos value will be set to the current orientation state
   * @param velocity        A pointer whos value will be set to the current velocity state
   * 
   * @return false if states are not initialized, true otherwise
  */
  bool get(Eigen::Vector3d      *position,
           Eigen::Quaterniond   *orientation,
           Eigen::Vector6d      *velocity);


  /**
   * @brief Get position state
   * 
   * @param position        A pointer whos value will be set to the current position state
   * 
   * @return false if states are not initialized, true otherwise
  */
  bool get(Eigen::Vector3d      *position);


  /**
   * @brief Get orientation state
   * 
   * @param orientation     A pointer whos value will be set to the current orientation state
   * 
   * @return false if states are not initialized, true otherwise
  */
  bool get(Eigen::Quaterniond   *orientation);


  /**
   * @brief Get orientation state in euler angles
   * 
   * @param orientation     A pointer whos value will be set to the euler orientation
   * 
   * @return false if states are not initialized, true otherwise
  */
  bool getEuler(Eigen::Vector3d *orientation);


  /**
   * @brief Set position, orientation and velocity states
   * 
   * @param position      A 3d vector containing the position to be set 
   * @param orientation   A quaternion containing the orientation to be set
   * @param velocity      A 6d vector containing the velocity to be set
   * 
   * @warning All state values are considered invalid until this function is called.
  */
  void set(const Eigen::Vector3d    &position,
           const Eigen::Quaterniond &orientation,
           const Eigen::Vector6d    &velocity);


  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:

  Eigen::Vector3d    m_position;      /** A 3d vector containing the position state*/
  Eigen::Quaterniond m_orientation;   /** A quaternion containing the orientation state*/
  Eigen::Vector6d    m_velocity;      /** A 6d vector containing the velocity state*/

  bool m_is_initialized;              /** A bool to indicate whether or not the states are initialized. False before set() is called*/
};

#endif  // VORTEX_CONTROLLER_STATE_H
