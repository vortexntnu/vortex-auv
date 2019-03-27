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
    dh_command.force.x = 0;
    dh_command.force.y = 0;
    dh_command.force.z = 0;

    dh_command.torque.x = 0;
    dh_command.torque.y = 0;
    dh_command.torque.z = 0;

    while(ros::ok()){
        dh_command.force.z = -this->height->calculate();
        std::cout << "Heave command" << dh_command.force.z << std::endl;
        pub_.publish(dh_command);
        ros::spinOnce();
        rate.sleep();
    }

    // helper variables
    ros::Rate r(1);
    bool success = true;

    // push_back the seeds for the DepthHold sequence
    feedback_.sequence.clear();
    feedback_.sequence.push_back(0);
    feedback_.sequence.push_back(1);

    // publish info to the console for the user
    ROS_INFO("%s: Executing, creating DepthHold sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

    // start executing the action
    for(int i=1; i<=goal->order; i++)
    {
        // check that preempt has not been requested by the client
        if (as_.isPreemptRequested() || !ros::ok())
        {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
        }
        feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
        // publish the feedback
        as_.publishFeedback(feedback_);
        // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
        r.sleep();
    }

    if(success)
    {
        result_.sequence = feedback_.sequence;
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        // set the action state to succeeded
        as_.setSucceeded(result_);
    }
}

void DepthHoldAction::stateEstimateCallback(const nav_msgs::Odometry &odometry_msgs){
    double error = static_cast<double>(odometry_msgs.pose.pose.position.z)*(-1) - this->default_height;
    std::cout <<"Error: "<<  error << std::endl;
    this->height->updateError(error);
}