## LOSController

#### Subscribes to:
* /guidance/los_data

#### Publishes to:
* /auv/thruster_manager/input

The purpose of the los_controller package is to provide the LOS guidance system
with a controller that can maintain a fixed heading and depth.

This is achieved by a 1D PID-controller and an integral backstepping controller. These two
controllers are used by the LOS guidance system respectively via the classes PIDRegulator and BacksteppingControl.



The controller parameters can be changed during runtime because of the dynamic reconfigure server that is present
in the los_controller_node script. The parameters can be found under /cfg/LOSController.cfg. To reconfigure the parameters, use
```
rosrun rqt_reconfigure rqt_reconfigure
```
and select the /los_controller node from the list of reconfigurable nodes.


**NOTE:** Currently, the dynamic reconfigure default values will be the parameters that the controller will end up using,
NOT the parameters in the (auv).yaml file.