#ifndef VORTEX_CONTROLLER_CONTROL_MODES_H
#define VORTEX_CONTROLLER_CONTROL_MODES_H

#include <string>

namespace ControlModes
{
enum ControlMode
{
  OPEN_LOOP           = 0,
  DEPTH_HOLD          = 1,
  HEADING_HOLD        = 2,
  DEPTH_HEADING_HOLD  = 3,
  OPEN_LOOP_RESTORING = 4,
  STAY_LEVEL          = 5,
  CONTROL_MODE_END    = 6
};
}  // namespace ControlModes
typedef ControlModes::ControlMode ControlMode;

inline std::string controlModeString(ControlMode control_mode)
{
  std::string s;
  switch (control_mode)
  {
    case ControlModes::OPEN_LOOP:
    s = "OPEN LOOP";
    break;

    case ControlModes::DEPTH_HOLD:
    s = "DEPTH HOLD";
    break;

    case ControlModes::HEADING_HOLD:
    s = "HEADING HOLD";
    break;

    case ControlModes::DEPTH_HEADING_HOLD:
    s = "DEPTH HEADING HOLD";
    break;

    case ControlModes::OPEN_LOOP_RESTORING:
    s = "OPEN LOOP RESTORING";
    break;

    case ControlModes::STAY_LEVEL:
    s = "STAY LEVEL";
    break;

    default:
    s = "INVALID CONTROL MODE";
    break;
  }
  return s;
}

#endif  // VORTEX_CONTROLLER_CONTROL_MODES_H
