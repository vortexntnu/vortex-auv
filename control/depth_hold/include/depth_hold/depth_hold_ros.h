#ifndef DEPTH_HOLD_ROS_H
#define DEPTH_HOLD_ROS_H

#include "depth_hold/DHpid.h"
#include <ros/ros.h>
#include <vortex_msgs/PropulsionCommand.h>
#include <vortex_msgs/RovState.h>
#include <geometry_msgs/Wrench.h>
#include <nav_msgs/Odometry.h>

#include <dynamic_reconfigure/server.h>
#include <depth_hold/DepthParamsConfig.h>


class DepthHold
{
    private:
    ros::NodeHandle m_nh;
    ros::Publisher pub;
    ros::Subscriber sub;
    dynamic_reconfigure::Server<depth_hold::DepthParamsConfig> server;
    //dynamic_reconfigure::Server<depth_hold::DepthParamsConfig>::CallbackType f;
    std::unique_ptr<DHpid> height;
    double default_height = 1.0;

    public:
    DepthHold(ros::NodeHandle m_nh);
    ~DepthHold();

    void stateEstimateCallback(const nav_msgs::Odometry &odometry_msgs);
    void configCallback(const depth_hold::DepthParamsConfig &config, uint32_t level);
    void spin();
};

#endif
