#include <depth_hold/DHpid.h>
#include <depth_hold/depth_hold_ros.h>
#include <ros/ros.h>
#include <vortex_estimator/simple_estimator.h>
#include <vortex_msgs/PropulsionCommand.h>

DepthHold::DepthHold(ros::NodeHandle nh) : m_nh(nh){
    sub = m_nh.subscribe("state_estimate", 1, &DepthHold::callback, this)
    pub = m_nh.advertise<vortex_msgs::PropulsionCommand>("/propulsion_command", 1)
    double dt = 0.1;
    double max = 1.0;
    double min = -1.0;
    double K_p = 0.0015;
    double K_d = 0.0013;
    double K_i = 0.0;

    height.reset(new DHpid(dt, max, min, K_p, K_d, K_i));
}

DepthHold::spin(){
    vortex_msgs::PropulsionCommand dh_command;

    ros::Rate rate(10);
    while(true){
        dh_command.motion[2] = this->height->calculate();
    }
    pub.publish(dh_command)
    ros::spinOnce();
    rate.sleep();
}

void DepthHold::callback(const <vortex_msgs::RovState> &estimated_height){
    double error = static_cast<double>(estimated_height.position.z) - this->default_height;
    this->height->updateError(error)
}