**#Control input merger - merging control input form all active controllers**

Subscribes on topics:
 -surge_input
 -sway_input
 -heave_input
 -roll_input
 -pitch_input
 -yaw_input
All of message type "Wrench" which consist of forces and tourqe in x,y,z


Publishes  on topic:
 -/manta/thruster_manager/input , -> Wrench message

**#Run**

Run the control input merge node by using the command:
```bash
 $ rosrun control_input_merger control_input_merger.py

