# Guidance interface

This node manages all guidance nodes and acts as a interface to them. It provides action servers as interface. Given a new goal, the interface will stop all current guidance activity before starting the requested one. 

Important: only use one action server at a time. Sending more than action at a time can cause undefined behaviour. 

## Input and output

Action servers that act as interface to guidance
* /guidance_interface/joystick_server (vortex_msgs/ControlModeAction)
* /guidance_interface/vel_server (vortex_msgs/SetVelocityAction)
* /guidance_interface/dp_server (move_base_msgs/MoveBaseAction)
* /guidance_interface/los_server (vortex_msgs/LosPathFollowingAction)

Action serves the interface uses
* dp_action_server (move_base_msgs/MoveBaseAction)
* los_action_server (vortex_msgs/LosPathFollowingAction)

Services the interface uses
* /joystick_guidance/activate_joystick_control (std_srvs/SetBool)
* /vel_guidance/set_velocity (vortex_msgs/SetVelocity)
* /vel_guidance/stop_guidance (std_srvs/Empty)
* /controller/controlmode_service (vortex_msgs/ControlMode)


## Parameters

* /guidance/interface/action_timeout (default=90) [s] time given for los and dp actions to complete
