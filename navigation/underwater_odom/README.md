# Underwater Odom 

This node takes data from the */depth/estimated* and */dvl/twist* topics
and combines it into a *nav_msgs::Odometry* message, which is then finally published to */dvl/odom*.

Note that the nav_msg is only published when new DVL data is published to */auv/dvl_twist*

## Input and Output

Subscibers
* __/depth/estimated__ estimated depth
* __/dvl/twist__ linear velocities from dvl

Publishers
* __/dvl/odom__ odom msg with linear velocities and depth
