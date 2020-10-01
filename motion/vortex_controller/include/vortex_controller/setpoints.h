#ifndef VORTEX_CONTROLLER_SETPOINTS_H
#define VORTEX_CONTROLLER_SETPOINTS_H

#include <Eigen/Dense>
#include "vortex/eigen_typedefs.h"

class Setpoints
{
public:
  Setpoints(const Eigen::Vector6d &wrench_scaling,
            const Eigen::Vector6d &wrench_max);
  bool update(const Eigen::Vector6d &command);
  bool get(Eigen::Vector6d *wrench);
  bool get(Eigen::Vector3d    *position,
           Eigen::Quaterniond *orientation);
  bool get(Eigen::Vector3d *position);
  bool get(Eigen::Quaterniond *orientation);
  bool getEuler(Eigen::Vector3d *orientation);
  void set(const Eigen::Vector3d    &position,
           const Eigen::Quaterniond &orientation);
  void set(const Eigen::Vector3d    &position);
  void set(const Eigen::Quaterniond &orientation);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
  Eigen::Vector6d    m_wrench;
  Eigen::Vector3d    m_position;
  Eigen::Quaterniond m_orientation;

  Eigen::Vector6d m_wrench_scaling;
  Eigen::Vector6d m_wrench_max;

  bool   m_wrench_is_valid;
  bool   m_pose_is_valid;
};

#endif  // VORTEX_CONTROLLER_SETPOINTS_H
