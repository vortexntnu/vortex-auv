# Ekf Filtering

This package provides functionality to map object in local camera fram to global frame in ROS2.

# Subscriptions

### Image Raw Topic
Subscipes to a `sensor_msgs::msg::Image` topic specified in the params file. This is the image on which aruco detection is performed.

### Camera Info Topic
The node sets default camera calibration parameters, but listens for a `sensor_msgs::msg::CameraInfo` topic to override the calibration parameters.
