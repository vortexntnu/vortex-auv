## Motion
Anything related to how the AUV physically moves in located in this folder, namely controllers and guidance systems, as well as interfaces to the mechanical/electrical components.


### Controllers
#### Compensates and stabilizes a system
* dp_controller
    * The dynamic positioning controller, which is implemented as a linear PID controller.

* local_pid
    * A pole-placement PID controller meant to be used with a setpoint in a local frame e.g. camera, body, etc.
  
* velocity_controller
    * A velocity controller using six one-dimensional PIDs with feed-forward term and integral windup protection. The control law includes compensation for restoring forces.

* velocity_controller
    * A velocity controller using six one-dimensional PIDs with feed-forward term and integral windup protection. The control law includes compensation for restoring forces.

* vtf_guidance_and_control
    * Virtual Target Following guidance and control module which sets up a target which follows a given path. The system will then follow the target's reference model using a MIMO DP controller.


### Guidance
#### Provides directional instructions to controllers

* guidance_interface
    * The interface for all the different guidance modules that the state machine uses to steer the AUV

* reference_model
    * Model for filtering the reference input for more optimal LOS and DP control.


### Others
#### Interfaces to physical units and misc

* thrust_merger
    * Linearly combines forces from multiple sources for multi-control-law operation 

* thruster_allocator
    * Calculates the specific thruster forces from a given thrust vector

* motion_launch
    * A container for the launch file that launches every required motion node. 
