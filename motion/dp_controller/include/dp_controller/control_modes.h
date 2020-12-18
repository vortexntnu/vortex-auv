/*   Written by Kristoffer Rakstad Solberg, Student
     Copyright (c) 2019 Manta AUV, Vortex NTNU.
     All rights reserved. */

#ifndef VORTEX_CONTROLLER_CONTROL_MODES_H
#define VORTEX_CONTROLLER_CONTROL_MODES_H

#include <string>


namespace ControlModes
{
enum ControlMode
{
  OPEN_LOOP           = 0,
  POSE_HOLD           = 1,
  HEADING_HOLD        = 2,
  DEPTH_HEADING_HOLD  = 3,
  DEPTH_HOLD          = 4,
  POSE_HEADING_HOLD   = 5,
  CONTROL_MODE_END    = 6
};
}  // namespace ControlModes
typedef ControlModes::ControlMode ControlMode;

/**
 * @brief Convert controlMode to a string representation
 * 
 * @param control_mode the control mode to be converted to a string
 * 
*/
inline std::string controlModeString(ControlMode control_mode)
{
  std::string s;
  switch (control_mode)
  {
    case ControlModes::OPEN_LOOP:
    s = "OPEN LOOP";
    break;

    case ControlModes::POSE_HOLD:
    s = "POSE HOLD";
    break;

    case ControlModes::HEADING_HOLD:
    s = "HEADING HOLD";
    break;

    case ControlModes::DEPTH_HEADING_HOLD:
    s = "DEPTH HEADING HOLD";
    break;

    case ControlModes::DEPTH_HOLD:
    s = "DEPTH HOLD";
    break;

    case ControlModes::POSE_HEADING_HOLD:
    s = "POSE HEADING HOLD";
    break;

    default:
    s = "INVALID CONTROL MODE";
    break;
  }
  return s;
}

#endif  // VORTEX_CONTROLLER_CONTROL_MODES_H
