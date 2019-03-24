#ifndef DEPTH_HOLD_ACTION_SERVER_ROS_H
#define DEPTH_HOLD_ACTION_SERVER_ROS_H

#include "depth_hold_action_server/DHpid.h"
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "depth_hold_action_server/DepthHoldAction.h"


class DepthHoldAction
{
    protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<depth_hold_action_server::DepthHoldAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    std::string action_name_;
    // create messages that are used to published feedback/result
    depth_hold_action_server::DepthHoldFeedback feedback_;
    depth_hold_action_server::DepthHoldResult result_;

    public:
    DepthHoldAction(std::string name);
    ~DepthHoldAction();

    void executeCB(const depth_hold_action_server::DepthHoldGoalConstPtr &goal);
};

#endif