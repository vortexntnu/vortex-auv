# Velocity Guidance

Simple node that recieves desired velocities through a service and publishes them with a predefined rate as a topic until a stop-service is called. 

## Input and output

Publishes:
* /desired_velocity (geometry_msgs/Twist)

Services:
* /vel_guidance/set_velocity (vortex_msgs/SetVelocity)
* /vel_guidance/stop_guidance (std_srvs/Empty)

## Setup and parameters

* __/vel_guidance/desired_velocity_topic__ set velocity publisher topic (default: /desired_velocity)
* __/vel_guidance/rate__ set control loop rate (default: 40)
