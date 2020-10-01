#include "vortex_controller/state.h"

State::State()
{
  m_position.setZero();
  m_orientation.setIdentity();
  m_velocity.setZero();
  m_is_initialized = false;
}

bool State::get(Eigen::Vector3d    *position,
                Eigen::Quaterniond *orientation)
{
  if (!m_is_initialized)
    return false;

  *position    = m_position;
  *orientation = m_orientation;
  return true;
}

bool State::get(Eigen::Vector3d    *position,
                Eigen::Quaterniond *orientation,
                Eigen::Vector6d    *velocity)
{
  if (!m_is_initialized)
    return false;

  *position    = m_position;
  *orientation = m_orientation;
  *velocity    = m_velocity;
  return true;
}

bool State::get(Eigen::Vector3d *position)
{
  if (!m_is_initialized)
    return false;

  *position = m_position;
}

bool State::get(Eigen::Quaterniond *orientation)
{
  if (!m_is_initialized)
    return false;

  *orientation = m_orientation;
}

bool State::getEuler(Eigen::Vector3d *orientation)
{
  if (!m_is_initialized)
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


void State::set(const Eigen::Vector3d    &position,
                const Eigen::Quaterniond &orientation,
                const Eigen::Vector6d    &velocity)
{
  m_position    = position;
  m_orientation = orientation;
  m_velocity    = velocity;

  m_is_initialized = true;
}
