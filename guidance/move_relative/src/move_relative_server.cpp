#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <actionlib/server/simple_action_server.h>
#include <move_relative/MoveRelativeAction.h>
#include <iostream>

class MoveRelativeAction
{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<move_relative::MoveRelativeAction> as_;
    std::string action_name_;
    move_relative::MoveRelativeGoal goal_;
    move_relative::MoveRelativeFeedback feedback_;
    move_relative::MoveRelativeResult result_;
    //ros::Subscriber sub_;
public:
    MoveRelativeAction(std::string name) :
        as_(nh_, name, false),
        action_name_(name)
    {
        as_.registerGoalCallback(boost::bind(&MoveRelativeAction::goalCB, this));
        as_.registerPreemptCallback(boost::bind(&MoveRelativeAction::preemtCB, this));
        as_.start();
    }

    ~MoveRelativeAction(void)
    {
    }

    void goalCB()
    {
        goal_.x = as_.acceptNewGoal()->x;
        goal_.y = as_.acceptNewGoal()->y;
        goal_.z = as_.acceptNewGoal()->z;
        goal_.angle = as_.acceptNewGoal()->angle;
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
    ros::init(argc, argv, "move_relative");

    MoveRelativeAction movement(ros::this_node::getName());
    ros::spin();
    return 0;
}
