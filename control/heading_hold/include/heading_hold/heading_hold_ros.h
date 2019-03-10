#ifndef HEADING_HOLD_ROS_H
#define HEADING_HOLD_ROS_H

#include "heading_hold/HHpid.h"
#include <ros/ros.h>
#include <vortex_msgs/PropulsionCommand.h>
#include <vortex_msgs/RovState.h>



class HeadingHold
{
    private:
    ros::NodeHandle m_nh;
    ros::Publisher pub;
    ros::Subscriber sub;

    double yaw_ref = 0.5;

    //Init a PID to control yaw
    std::unique_ptr<HHpid> yaw;

    public:
    HeadingHold(ros::NodeHandle m_nh);
    ~HeadingHold();

    void stateEstimateCallback(const vortex_msgs::RovState &msg);
    void spin();
};

#endif