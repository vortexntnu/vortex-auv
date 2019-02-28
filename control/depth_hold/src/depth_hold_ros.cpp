#include <depth_hold/DHpid.h>
#include <depth_hold/depth_hold_ros.h>
#include <ros/ros.h>
#include <vortex_estimator/simple_estimator.h>
#include <vortex_msgs/PropulsionCommand.h>
#include <vortex_msgs/RovState.h>
#include <iostream>

//Constructor
DepthHold::DepthHold(ros::NodeHandle nh) : m_nh(nh){
    sub = m_nh.subscribe("state_estimate", 1, &DepthHold::stateEstimateCallback, this);
    pub = m_nh.advertise<vortex_msgs::PropulsionCommand>("heave_input", 1);
    double dt = 0.1;
    double max = 1.0;
    double min = -1.0;
    double K_p = 0.015;
    double K_d = 0.013;
    double K_i = 0.01;

    height.reset(new DHpid(dt, max, min, K_p, K_d, K_i));
}

//Destructor
DepthHold::~DepthHold(){}


void DepthHold::spin()
{
    vortex_msgs::PropulsionCommand dh_command;
    dh_command.control_mode.resize(6);
    dh_command.control_mode[1]=1;
    pub.publish(dh_command);

    ros::Rate rate(10);
    while(ros::ok()){
        dh_command.motion[2] = -this->height->calculate();
        std::cout << "Heave command" << dh_command.motion[2] << std::endl;
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