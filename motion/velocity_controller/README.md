# Velocity Controller

A velocity controller using six one-dimensional PIDs with feed-forward term and integral windup protection. The control law includes compensation for restoring forces. 

## Input and output

Subscribes to:
* /odometry/filtered (nav_msgs::Odometry)
* /controller/desired_velocity (geometry_msgs::Twist)

Publishes:
* /thrust/desired (geometry_msgs::Wrench)

Services:
* velocity_controller/reset_pid (std_srvs::Empty)
* velocity_controller/set_gains (vortex_msgs::SetPidGains)

## Setup and parameters

* __/velocity_controller/odometry_topic__ set odometry subscriber topic. (default: /odometry/filtered)
* __/velocity_controller/thrust_topic__ set thrust publisher topic (default: /thrust/desired)
* __/velocity_controller/desired_velocity_topic__ set desired velocity subscriber topic (default: /controller/desired_velocity)
* __physical/weight__ how much the drone weights [kg]
* __physical/bouyancy__ buoyancy of the drone [kg]
* __physical/center_of_bouyancy__ distance from center of origin to center of buoyancy [mm]
* __physical/center_of_mass__ distance from center of origin to center of mass/gravity [mm]
* __controllers/velocity_controller/P_gains__ Array of P gains in (surge, sway, heave, roll, pitch, yaw)
* __controllers/velocity_controller/I_gains__ Array of I gains in (surge, sway, heave, roll, pitch, yaw)
* __controllers/velocity_controller/D_gains__ Array of D gains in (surge, sway, heave, roll, pitch, yaw)
* __controllers/velocity_controller/F_gains__ Array of F gains (feed-forward) in (surge, sway, heave, roll, pitch, yaw)
* __controllers/velocity_controller/integral_windup_limit__ Integral windup limit in (surge, sway, heave, roll, pitch, yaw)
