# The Line-Of-Sight guidance module

## Overview
The LOS guidance node is a guidance system that is utilized for the AUV to move along straight line segments.
The node takes in odometry data, and publishes data that the autopilot controller node will use to calculate control vectors.

## Coming soon
An action client will be implemented in this node. The client will receive commands from an action server in the FSM system. Only when the LOS node is addressed from the FSM will it produce data messages for the autopilot controller.
<< A diagram showing the setup will also be put here >>
