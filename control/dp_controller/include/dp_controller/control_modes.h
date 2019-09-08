/*   Written by Kristoffer Rakstad Solberg, Student
     and Morten Fyhn Amundsen, Student
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
  POSE_HEADING_HOLD   = 4,
  STAY_LEVEL          = 5,
  CONTROL_MODE_END    = 6
};
}  // namespace ControlModes
typedef ControlModes::ControlMode ControlMode;

/*
Control mode
Set all elements false to keep current mode.
Set one element true to switch mode. Its index determines the mode,
as defined in the control_modes.h.
bool[] control_mode */

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

    case ControlModes::POSE_HEADING_HOLD:
    s = "POSE HEADING HOLD";
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
