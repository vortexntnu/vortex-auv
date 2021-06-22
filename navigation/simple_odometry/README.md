# Simple Odometry

Node that combines measurements from IMU and DVL into a simple odometry publisher. Positions in x and y are estimated by euler integration.

## Input and output

Subscribers
* /auv/imu (sensor_msgs/Imu)
* /auv/dvl (nav_msgs/Odometry)

Publishers
* /auv/odom (nav_msgs/Odometry)

## Parameters

* __simple_odom/imu_topic__ topic for imu subscriber
* __simple_odom/dvl_topic__ topic for dvl subscriber
* __simple_odom/odom_topic__ topic for odometry publisher
* __simple_odom/update_rate__ rate of spin loop
