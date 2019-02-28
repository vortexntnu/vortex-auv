#include <depth_hold/DHpid.h>
#include <depth_hold/depth_hold_ros.h>
#include <ros/ros.h>
#include <vortex_estimator/simple_estimator.h>
#include <vortex_msgs/PropulsionCommand.h>
#include <vortex_msgs/RovState.h>
#include <iostream>
#include <geometry_msgs/Wrench.h>

//Constructor
DepthHold::DepthHold(ros::NodeHandle nh) : m_nh(nh){
    sub = m_nh.subscribe("state_estimate", 1, &DepthHold::stateEstimateCallback, this);
    pub = m_nh.advertise<geometry_msgs::Wrench>("heave_input", 1);
    double dt = 0.1;
    double max = 40.0;
    double min = -40.0;
    double K_p = 1.5;
    double K_d = 0.13;
    double K_i = 0.05;

    height.reset(new DHpid(dt, max, min, K_p, K_d, K_i));
}

//Destructor
DepthHold::~DepthHold(){}


void DepthHold::spin()
{
    geometry_msgs::Wrench dh_command;
    ros::Rate rate(10);
    dh_command.force.x = 0;
    dh_command.force.y = 0;
    dh_command.force.z = 0;
    
    dh_command.torque.x = 0;
    dh_command.torque.y = 0;
    dh_command.torque.z = 0;
    
    while(ros::ok()){
        dh_command.force.z = -this->height->calculate();
        std::cout << "Heave command" << dh_command.force.z << std::endl;
    pub.publish(dh_command);
    ros::spinOnce();
    rate.sleep();
    }
}

void DepthHold::stateEstimateCallback(const vortex_msgs::RovState &estimated_height){
    double error = static_cast<double>(estimated_height.pose.position.z) - this->default_height;
    std::cout <<"Error: "<<  error << std::endl;
    this->height->updateError(error);
}