## Navigation launch

This package is a container for the launch file that launches every required node in the 'navigation' category, namely the extended kalman filter and the underwater odometry node. When the real AUV system is launched, additional nodes/drivers for the DVL and IMU are launched. The navigation.launch file is launched by the top-level launchfile in the auv_setup package.