**#Control input merger - merging control input form all active controllers**

Subscribes on topics:
 -surge_input
 -sway_input
 -heave_input
 -roll_input
 -pitch_input
 -yaw_input
All of message type "Wrench" which consist of forces and tourqe in x,y,z


Publishes A

**#Run**

Run the depth hold node by using the command:
```bash
 $ rosrun depth_hold depth_hold_node
```
You can observe how Manta performs by listening to the topic /state_estimate
```bash
  $ rostopic echo /state_estimate
```
and look at 
```
  position.z
```
