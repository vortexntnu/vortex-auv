## Motion
Anything related to how the AUV physically moves in located in this folder, namely controllers and guidance systems, as well as interfaces to the mechanical/electrical components.


### Controllers
<details>
<summary>The currently implemented controllers:</summary>

* los_controller
    * An integral backstepping controller used by the los_guidance system

* dp_controller
    * The dynamic positioning controller, which is implemented as a nonlinear PID controller.

</details>

### Guidance
<details>
<summary>The currently implemented guidance modules:</summary>

* interface
    * The interface for all the different guidance modules that the state machine uses to steer the AUV

* los_guidance
    * A straight-line guidance module


* dp_guidance
    * The state machine technically can reference the dp controller directly, but this module is in place to fully separate the controllers and state machine.

</details>



### Others
<details>
<summary>Interfaces to physical units and misc:</summary>

* reference_model
    * Reference model calculation used in the LOS guidance system.
    
* mcu_interface
    * The interface between the ROS system and a multi-purpose MicroController Unit (MCU) in the AUV.

* thruster_interface
    * The interface between the control system and the ESCs, i.e. a board that can produce PWM signals.

* vortex_allocator
    * Calculates the specific thruster forces from a given thrust vector

* motion_launch
    * A container for the launch file that launches every required motion node. 

</details>