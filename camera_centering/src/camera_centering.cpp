#include <ros/ros.h>
#include <vortex_msgs/PropulsionCommand.h>
#include <vortex_msgs/CameraObjectInfo>
#include <camera_centering/camerapid.h>

int main(int argc, char* argv[])
{
ros::init(argc, argv, "camera_centering");
ros::NodeHandle nh;
ros::Publisher pub;
ros::Subscriber sub;
pub = nh.advertise<vortex_msgs::PropulsionCommand>("/propulsion_command",1);
sub = nh.subscribe("/camera_object_info", 1 /*callback, this */);
//sub.pos_x            (int16) object pos
//sub.pos_y            (int16) object pos
//sub.frame_width      (int16) camera feed width
//sub.frame_height     (int16) camera feed height
//sub.confidence       (float64) between 0.0 and 1.0)


//Initialize pid in x and y
PID::PID pidx = PID(0.1, 1, -1, 0.1, 0.01, 0.5);
PID::PID pidy = PID(0.1, 1, -1, 0.1, 0.01, 0.5);


//Initialize depth hold mode
vortex_msgs::PropulsionCommand propulsion;
propulsion.control_mode.resize(6);
propulsion.control_mode[1]=1;
pub.publish(propulsion);


// 10 Hz
ros::Rate rate(10);
while(true){
  if (sub.confidence > 0.5){

    //Objects to the left of center has negative error
    pidx.updateError(sub.pos_x - sub.frame_width/2);
    //Objects over center has negative error
    pidy.updateError(sub.pos_y - sub.frame_height/2);

    //Turn right/left
    propulsion.motion[5] = pidx.calculate();

    //Ascend/Descend
    propulsion.motion[2] = pidy.calculate();
  }
  else{
    //Stop propulsion
    propulsion.motion[5] = 0;
    propulsion.motion[2] = 0;
  }

  pub.publish(propulsion);
  rate.sleep();
}
}
