#include <vortex_msgs/camerapid.h>

Class CameraCentering{
private:
PID pidx;
PID pidy;
ros::NodeHandle nh;
ros::Publisher pub;
ros::Subscriber sub;

//sub.pos_x            (int16) object pos
//sub.pos_y            (int16) object pos
//sub.frame_width      (int16) camera feed width
//sub.frame_height     (int16) camera feed height
//sub.confidence       (float64) between 0.0 and 1.0)


public:
explicit CameraCentering(ros::NodeHandle nh);
void cameraobjectcallback(vortex_msgs::CameraObjectInfo);
void spin();

}












