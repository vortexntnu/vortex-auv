#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <camera_aiming/AimingAction.h>
#include <vortex_msgs/CameraObjectInfo.h>
#include "vortex_msgs/MoveRelativeAction.h"
#include <camera_aiming/Camerapid.h>
#include <iostream>
#include <memory>

class AimingAction
{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<camera_aiming::AimingAction> as_;
    std::string action_name_;
    std::string goal_;
    camera_aiming::AimingFeedback feedback_;
    camera_aiming::AimingResult result_;

    ros::Subscriber sub_;
    std::unique_ptr<Camerapid> pidx;
    std::unique_ptr<Camerapid> pidy;
    double confidence;

    vortex_msgs::MoveRelativeGoal movementGoal_;


public:
    AimingAction(std::string name) :
        as_(nh_, name, false),
        action_name_(name)
    {
        as_.registerGoalCallback(boost::bind(&AimingAction::goalCB, this));
        as_.registerPreemptCallback(boost::bind(&AimingAction::preemtCB, this));
        as_.start();

        sub_ = nh_.subscribe("/gate_midpoint", 1, &AimingAction::cameraInfoCB, this);
        pidx.reset(new Camerapid(0.1,5,-5,0.0015,0.0013,0.0001));
        pidy.reset(new Camerapid(0.1,1,-1,0.005,0.05,0.0));
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

    void cameraInfoCB(const vortex_msgs::CameraObjectInfo& info)
    {
        this->confidence = info.confidence;
        //Objects to the left of center has negative error
        this->pidx->updateError(info.pos_x - info.frame_width/2);
        //Objects over center has negative error
        this->pidy->updateError(info.pos_y - info.frame_height/2);
    }

    void spin()
    {
        //Create move_relative action client
        actionlib::SimpleActionClient<vortex_msgs::MoveRelativeAction> ac("move_relative");
        ROS_INFO("Waiting for action server to start.");
        ac.waitForServer();
        // 10 Hz
        ros::Rate rate(10);
        while(ros::ok())
        {
            if (confidence > 0.8)
            {
                //Turn right/left
                movementGoal_.angle = this->pidx->calculate();
                //Ascend/Descend
                movementGoal_.z = this->pidy->calculate();
                ac.sendGoal(movementGoal_);
            }
            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_aiming");

    AimingAction aiming(ros::this_node::getName());
    aiming.spin();
    return 0;
}
