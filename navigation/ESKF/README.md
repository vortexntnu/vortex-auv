# Error State Kalman Filter

State estimator that estimates position, orientation (quaternions) and linear velocities by comparing double integrations of IMU measurements with DVL mesurements. 

## Inputs and Outputs

Publishes

- ESKF/pose (nav_msgs::odometry)
- BiasAndGravity (nav_msgs::odometry)
- nis_dvl (nav_msgs::odometry)
- nis_pressureZ (nav_msgs::odometry)

Subscribes to 

- IMU (sensor_msgs::Imu)
- Pressure (nav_msgs::odometry)
- DVL (nav_msgs::odometry)

## Setup instructions

### Launch

can be found in apps/ros/launch

See Overwater_30_min_test.launch for a good example.

output="screen" will print all internal prints as well as those from ROS.  
clear_params="true": important. Clears all previous params with the same names. 


### Parameters

can be found in apps/ros/parameters

- publish rate
- publish in ENU
- imu_topic
- R_acc (output noise cov. matrix of accelerometer)
- R_gyro(output noise cov. matrix of gyroscope)
- sr_accelorometer_to_NED (static transform from IMU-frame to BODY-frame)
- sr_gyro_to_NED (static transform from IMU-frame to BODY-frame)
- sr_accelerometer_alignment (alignment error of IMU placement)
- sr_gyro_alignment (alignment error of IMU placement)
- st_inc (inconometer alignemt expressed as rotation matrix. TODO: change to roll, pitch, yaw format)
- R_accBias (bias noise cov. matrix of accelerometer)
- R_gyroBias (bias noise cov. matrix of accelerometer)
- p_gyroBias ()
- p_accBias ()
- dvl_topic
- R_dvl (output noise cov. matrix of dvl)
- sr_dvl_to_NED (orientation and placement of sensor in BODY) 
- sr_dvl_alignment (error in placement)
- pressureZ_topic
- R_pressureZ (measurement noise)
- initial_pose (where it starts in world frame. Includes biases and gravity estimates)
- initial_covariance (of all states, in same order as in initial pose)
