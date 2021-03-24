## Line-Of-Sight guidance module
The LOS guidance node is a guidance system that is utilized for the AUV to move along straight line segments.
The node takes in odometry data and a point in the global coordinate frame, and publishes data that the los_controller controller node will use to calculate control vectors.


The system parameters can be changed during runtime because of the dynamic reconfigure server that is present
in the los_guidance_backstepping script. The parameters can be found under /cfg/LOS.cfg. To reconfigure the parameters, use
```
rosrun rqt_reconfigure rqt_reconfigure
```
and select the /losnode from the list of reconfigurable nodes.

