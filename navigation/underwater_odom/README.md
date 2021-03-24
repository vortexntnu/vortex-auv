## Odometry

This package creates the odometry node, which takes data from the */auv/pressure* and */auv/dvl_twist* topics
and combines it into a *nav_msgs::Odometry* message, which is then finally published to */auv/odom*.

Note that the pressure data is converted into a depth reading before added to the nav_msg.
Also note that the nav_msg is only published when new DVL data is published to */auv/dvl_twist*
