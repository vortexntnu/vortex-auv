#ifndef DEPTH_HOLD_ROS_H
#define DEPTH_HOLD_ROS_H

#include "depth_hold/DHpid.h"
#include <ros/ros.h>
#include <vortex_msgs/PropulsionCommand.h>
#include <vortex_msgs/RovState.h>



class DepthHold
{
    private:
    ros::NodeHandle m_nh;
    ros::Publisher pub;
    ros::Subscriber sub;

    double default_height = 1.0;
    std::unique_ptr<DHpid> height;

    public:
    DepthHold(ros::NodeHandle m_nh);
    ~DepthHold();

    void stateEstimateCallback(const vortex_msgs::RovState &estimated_height);
    void spin();
};

#endif