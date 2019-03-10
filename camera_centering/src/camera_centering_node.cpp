#include "camera_centering/camera_centering_ros.h"
#include <ros/ros.h>

int main(int argc, char **argv)
{
ros::init(argc, argv, "camera_centering");
ros::NodeHandle nh;
CameraCentering cameracentering(nh);
cameracentering.spin();
return 0;
}
