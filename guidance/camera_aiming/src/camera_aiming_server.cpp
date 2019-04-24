#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <actionlib/server/simple_action_server.h>
#include <camera_aiming/AimingAction.h>
#include <iostream>

class AimingAction
{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<camera_aiming::AimingAction> as_;
    std::string action_name_;
    std::string goal_;
    camera_aiming::AimingFeedback feedback_;
    camera_aiming::AimingResult result_;
    //ros::Subscriber sub_;
public:
    AimingAction(std::string name) :
        as_(nh_, name, false),
        action_name_(name)
    {
        as_.registerGoalCallback(boost::bind(&AimingAction::goalCB, this));
        as_.registerPreemptCallback(boost::bind(&AimingAction::preemtCB, this));
        as_.start();
    }

    ~AimingAction(void)
    {
    }

    void goalCB()
    {
        goal_ = as_.acceptNewGoal()->goal;
    }

    void preemtCB()
    {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        as_.setPreempted();
    }

    // void cameraInfoCB(const vortex_msgs::CameraObjectInfo& msg)
    // {
    //     //send wanted relative movements
    // }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_aiming");

    AimingAction aiming(ros::this_node::getName());
    ros::spin();
    return 0;
}
