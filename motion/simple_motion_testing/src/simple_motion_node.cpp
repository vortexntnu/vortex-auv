
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>


#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Vector3.h"

int main(int argc, char **argv)

{
    ros::init(argc,argv,"simple_motion_node");
    ros::NodeHandle nh;

    ros::Publisher wrench_pub = nh.advertise<geometry_msgs::Wrench>("/auv/thruster_manager/input", 100);

    ros::Rate loop_rate(10);

    //generate hard-coded wrench message
  
    geometry_msgs::Vector3 force;
    force.x = 10.0;
    force.y = 0.0;
    force.z = 0.0;
    geometry_msgs::Vector3 torque;
    torque.x = 0.0;
    torque.y = 0.0;
    torque.z = 0.0;
    geometry_msgs::Wrench msg;
    msg.force = force;
    msg.torque = torque;

    //publish wrench message to topic /auv/thruster_manager/input


    while(1){
        wrench_pub.publish(msg);
        sleep(10);
    }

    ros::spin();

    return 0;
}