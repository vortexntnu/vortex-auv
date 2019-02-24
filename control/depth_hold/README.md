#Depth hold Package to have constant depth

Subscribes to the topic 'state_estimate', which calculates the height
from the pressure. 

Controles heave by a simple PID controller by currently publishing
directly to /propulsion_command.
