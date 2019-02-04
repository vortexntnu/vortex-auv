#include <ros/ros.h>
#include <vortex_msgs/PropulsionCommand.h>
#include <vortex_msgs/CameraObjectInfo.h>
#include <camera_centering/Camerapid.h>
#include <camera_centering/camera_centering_ros.h>
#include <iostream>

CameraCentering::CameraCentering(ros::NodeHandle nh) : m_nh(nh)
{
pub = m_nh.advertise<vortex_msgs::PropulsionCommand>("/propulsion_command",1);
sub = m_nh.subscribe("/camera_object_info", 1, &CameraCentering::cameraobjectcallback, this);

//pidx* = new PID(0.1,1,-1,0.01,0.1,0.5);
//pidy* = new PID(0.1,1,-1,0.01,0.1,0.5);
pidx.reset(new Camerapid(0.1,1,-1,0.01,0.1,0.5));
pidy.reset(new Camerapid(0.1,1,-1,0.01,0.1,0.5));
}


CameraCentering::~CameraCentering()
{
}

void CameraCentering::spin()
{


//Initialize depth hold mode
vortex_msgs::PropulsionCommand propulsion;
propulsion.control_mode.resize(6);
propulsion.control_mode[1]=1;
pub.publish(propulsion);


// 10 Hz
ros::Rate rate(10);
while(true){
  //if (sub.confidence > 0.5){
    //Turn right/left
    propulsion.motion[5] = this->pidx->calculate();

    //Ascend/Descend
    propulsion.motion[2] = this->pidy->calculate();
  //}
  //else{
    //Stop propulsion
    //propulsion.motion[5] = 0;
    //propulsion.motion[2] = 0;
  //}

  pub.publish(propulsion);
  rate.sleep();
std::cout<< "HEIEHI" <<std::endl;
}
}

void CameraCentering::cameraobjectcallback(const vortex_msgs::CameraObjectInfo &info)
{
  //Objects to the left of center has negative error
  this->pidx->updateError(info.pos_x - info.frame_width/2);
  //Objects over center has negative error
  this->pidy->updateError(info.pos_y - info.frame_height/2);
}






