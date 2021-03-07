
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>


#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Vector3.h"

class subscribe_and_publish{
public:
    subscribe_and_publish(){
        wrench_pub = nh.advertise<geometry_msgs::Wrench>("/auv/thruster_manager/input", 100);
        odom_sub = nh.subscribe("/odometry/filtered", 100, &subscribe_and_publish::odometry_callback,this);
    }

    void odometry_callback(const nav_msgs::Odometry & position_msg){
        std::cout << position_msg.pose.pose.position.x << "\n";
        //geometry_msgs::Wrench msg;
        //msg.force.x = 10.0;//K_p*(reference-position_msg.pose.pose.position.x);
       // wrench_pub.publish(msg);
        sleep(1);
    }
private:
    ros::NodeHandle nh;
    ros::Publisher wrench_pub;
    ros::Subscriber odom_sub;
    float reference = 1.0;
    float K_p = 3;      
};



// void odometry_callback(const nav_msgs::Odometry & position_msg){
//     std::cout << position_msg.pose.pose.position.x << "\n";
//     //msg.force.x = 10.0;//K_p*(reference-position_msg.pose.pose.position.x);
//     //sleep(1);
    
// }

int main(int argc, char **argv)
{
    geometry_msgs::Wrench msg;

    ros::init(argc,argv,"simple_motion_node");
    
    // ros::NodeHandle nh;
    // ros::Publisher wrench_pub = nh.advertise<geometry_msgs::Wrench>("/auv/thruster_manager/input", 100);
    // ros::Subscriber odom_sub = nh.subscribe("/odometry/filtered", 100, &odometry_callback);

    //ros::Rate loop_rate(10);

    subscribe_and_publish P_controller;

    //publish wrench message to topic /auv/thruster_manager/input

    // msg.force.x = 10;
    // sleep(5);
    // wrench_pub.publish(msg);
    // sleep(5);
    // msg.force.x = -10;
    // wrench_pub.publish(msg);
    // sleep(5);
    // msg.force.x = 0;
    // wrench_pub.publish(msg);



    ros::spin();

    return 0;
}