#ifndef DEPHT_HOLD_ROS_H
#define DEPHT_HOLD_ROS_H

#include <ros/ros.h>
#include <depth_hold/DHpid.h>
#include <vortex_msgs/PropulsionCommand.h>
#include <vortex_estimator/simple_estimator.h>
#include <DHpid.h>

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

    void callback();
    void spin();
};

#endif