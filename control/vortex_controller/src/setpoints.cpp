#include "vortex_controller/setpoints.h"

Setpoints::Setpoints(const Eigen::Vector6d &wrench_scaling,
                     const Eigen::Vector6d &wrench_max)
: m_wrench_scaling(wrench_scaling),
  m_wrench_max(wrench_max)
{
  m_wrench.setZero();
  m_position.setZero();
  m_orientation.setIdentity();

  m_wrench_is_valid = false;
  m_pose_is_valid   = false;
}

bool Setpoints::update(const Eigen::Vector6d &command)
{
  for (int i = 0; i < 6; ++i)
    m_wrench(i) = m_wrench_scaling(i) * m_wrench_max(i) * command(i);
  m_wrench_is_valid = true;

  return true;
}

bool Setpoints::get(Eigen::Vector6d *wrench)
{
  if (!m_wrench_is_valid)
    return false;

  *wrench = m_wrench;
  return true;
}

bool Setpoints::get(Eigen::Vector3d    *position,
                    Eigen::Quaterniond *orientation)
{
  if (!m_pose_is_valid)
    return false;

  *position    = m_position;
  *orientation = m_orientation;
  return true;
}

bool Setpoints::get(Eigen::Vector3d *position)
{
  if (!m_pose_is_valid)
    return false;

  *position    = m_position;
}

bool Setpoints::get(Eigen::Quaterniond *orientation)
{
  if (!m_pose_is_valid)
    return false;

  *orientation = m_orientation;
}

bool Setpoints::getEuler(Eigen::Vector3d *orientation)
{
  if (!m_pose_is_valid)
    return false;

  const double w = m_orientation.w();
  const double x = m_orientation.x();
  const double y = m_orientation.y();
  const double z = m_orientation.z();

  Eigen::Vector3d euler_orientation;

  euler_orientation[0] = std::atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y));  // ROLL
  euler_orientation[1] = std::asin(2*(w*y - x*z));                      // PITCH
  euler_orientation[2] = std::atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z));  // YAW

  *orientation = euler_orientation;
}

void Setpoints::set(const Eigen::Vector3d    &position,
                    const Eigen::Quaterniond &orientation)
{
  m_position    = position;
  m_orientation = orientation;

  m_pose_is_valid = true;
}

void Setpoints::set(const Eigen::Vector3d &position)
{
  m_position   = position;
}

void Setpoints::set(const Eigen::Quaterniond &orientation)
{
  m_orientation = orientation;
}

