#ifndef CAMERA_CENTERING_ROS_H
#define CAMERA_CENTERING_ROS_H

#include <camera_centering/Camerapid.h>
#include <ros/ros.h>
#include <vortex_msgs/CameraObjectInfo.h>
#include <vortex_msgs/PropulsionCommand.h>

class CameraCentering
{
private:
ros::NodeHandle m_nh;
ros::Publisher pub;
ros::Subscriber sub;

//sub.pos_x            (int16) object pos
//sub.pos_y            (int16) object pos
//sub.frame_width      (int16) camera feed width
//sub.frame_height     (int16) camera feed height
//sub.confidence       (float64) between 0.0 and 1.0)

//Camerapid pidx = Camerapid(0.1,1,-1,0.01,0.1,0.5);
//Camerapid pidy = Camerapid(0.1,1,-1,0.01,0.1,0.5);
std::unique_ptr<Camerapid> pidx;
std::unique_ptr<Camerapid> pidy;
double confidence;

public:
CameraCentering(ros::NodeHandle nh);
~CameraCentering();
void cameraobjectcallback(const vortex_msgs::CameraObjectInfo &info);
void spin();

};

#endif
