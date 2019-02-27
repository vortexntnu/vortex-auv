#Depth hold Package to have constant depth

Subscribes to the topic 'state_estimate', which calculates the height
from the pressure. 

Controls heave by a simple PID controller by currently publishing
directly to /propulsion_command.

#Run
Run the depth hold node by using the command:

  rosrun depth_hold depth_hold_node

You can observe how Manta performs by listening to the topic /state_estimate

  rostopic echo /state_estimate

and look at 

  position.z
