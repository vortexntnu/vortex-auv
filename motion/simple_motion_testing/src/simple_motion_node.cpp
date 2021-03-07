
/**
* @file
* @brief A simple one-dimentional P-controller for AUV x position (global frame).
*/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <iostream>


#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Vector3.h"


/**
 * @brief the subcribe_and_publish class
 * 
 * This class handles all the publishing and subscribing to ros topics, as well as the implementation of the P-controller.
*/
class subscribe_and_publish{
public:

  /**
   * @brief subscribe_and_publish class constructor. Advertises and subscribes, as well as sets default reference value.
  */
    subscribe_and_publish(){
        wrench_pub = nh.advertise<geometry_msgs::Wrench>("/auv/thruster_manager/input", 100);
        ref_pub = nh.advertise<std_msgs::Float64>("simple_reference", 100);
        odom_sub = nh.subscribe("/odometry/filtered", 100, &subscribe_and_publish::odometry_callback,this);
        ref_sub = nh.subscribe("simple_reference", 100, &subscribe_and_publish::reference_callback,this);
        
        reference = 0.0;
    }

    /**
    * @brief callback function for the odom_sub subscriber. Generates and publishes suitable Wrench message to AUV.
    * @param position_msg A nav_msgs::Odometry message containing the current filtered odometry for the AUV
    */
    void odometry_callback(const nav_msgs::Odometry & position_msg){
        //std::cout << position_msg.pose.pose.position.x << "\n";
        msg.force.x = K_p*(reference-position_msg.pose.pose.position.x);
        wrench_pub.publish(msg);
    }

    /**
    * @brief callback function for the ref_sub subscriber. Updates private variable: reference.
    * @param ref_msg A  std_msgs::Float64 message containing the desired x position of the AUV.
    */
    void reference_callback(const std_msgs::Float64 & ref_msg){
        this->reference = ref_msg.data;
    }
private:
    ros::NodeHandle nh;
    ros::Publisher wrench_pub;
    ros::Publisher ref_pub;
    ros::Subscriber odom_sub;
    ros::Subscriber ref_sub;
    geometry_msgs::Wrench msg;
    float reference;
    float K_p = 3;      
}; 


/**
*@brief the int main() function initialises the ROS node and creates an instance of the subscribe_and_publish class.
*/
int main(int argc, char **argv)
{

    ros::init(argc,argv,"simple_motion_node");

    subscribe_and_publish P_controller;

    ros::spin();

    return 0;
}