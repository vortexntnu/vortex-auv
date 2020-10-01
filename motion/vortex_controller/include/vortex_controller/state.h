#ifndef VORTEX_CONTROLLER_STATE_H
#define VORTEX_CONTROLLER_STATE_H

#include <Eigen/Dense>
#include "vortex/eigen_typedefs.h"

class State
{
public:
  State();
  bool get(Eigen::Vector3d      *position,
           Eigen::Quaterniond   *orientation);
  bool get(Eigen::Vector3d      *position,
           Eigen::Quaterniond   *orientation,
           Eigen::Vector6d      *velocity);
  bool get(Eigen::Vector3d      *position);
  bool get(Eigen::Quaterniond   *orientation);
  bool getEuler(Eigen::Vector3d *orientation);
  void set(const Eigen::Vector3d    &position,
           const Eigen::Quaterniond &orientation,
           const Eigen::Vector6d    &velocity);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
  Eigen::Vector3d    m_position;
  Eigen::Quaterniond m_orientation;
  Eigen::Vector6d    m_velocity;

  bool m_is_initialized;
};

#endif  // VORTEX_CONTROLLER_STATE_H
