#include <depth_hold_action_server/DHpid.h>
#include <depth_hold_action_server/depth_hold_action_server_ros.h>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "depth_hold_action_server/DepthHoldAction.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Wrench.h>
#include <iostream>

DepthHoldAction::DepthHoldAction(std::string name) :
    as_(nh_, name, boost::bind(&DepthHoldAction::executeCB, this, _1), false),
    action_name_(name)
    {
        as_.start();

        sub_ = nh_.subscribe("/odometry/filtered", 1, &DepthHoldAction::stateEstimateCallback, this);

        pub_ = nh_.advertise<geometry_msgs::Wrench>("heave_input", 1);
        double dt = 0.1;
        double max = 40.0;
        double min = -40.0;
        double K_p = 1.5;
        double K_d = 0.13;
        double K_i = 0.05;

        height.reset(new DHpid(dt, max, min, K_p, K_d, K_i));
    }


DepthHoldAction::~DepthHoldAction()
{
}

void DepthHoldAction::executeCB(const depth_hold_action_server::DepthHoldGoalConstPtr &goal)
{
    geometry_msgs::Wrench dh_command;
    ros::Rate rate(10);
    bool success = true;

    this->goal_depth = goal->depth;
    ROS_INFO("%s: Executing Depth hold with goal depth %i", action_name_.c_str(), goal->depth);

    while(ros::ok()){
        // check that preempt has not been requested by the client
        if (as_.isPreemptRequested())
        {
            ROS_INFO("%s: Preempted", action_name_.c_str());
            // set the action state to preempted
            as_.setPreempted();
            success = false;
            break;
        }
        dh_command.force.z = -this->height->calculate();
        feedback_.current_depth = 50;
        as_.publishFeedback(feedback_);
        //std::cout << "Heave command" << dh_command.force.z << std::endl;
        pub_.publish(dh_command);
        ros::spinOnce();
        rate.sleep();
    }

    if(success)
    {
        result_.success = success;
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        // set the action state to succeeded
        as_.setSucceeded(result_);
    }
}

void DepthHoldAction::stateEstimateCallback(const nav_msgs::Odometry &odometry_msgs){
    double error = static_cast<double>(odometry_msgs.pose.pose.position.z)*(-1) - this->goal_depth;
    //std::cout <<"Error: "<<  error << std::endl;
    this->height->updateError(error);
}