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

  DepthHoldAction(std::string name) :
    as_(nh_, name, boost::bind(&DepthHoldAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~DepthHoldAction(void)
  {
  }

  void executeCB(const depth_hold_action_server::DepthHoldGoalConstPtr &goal)
  {
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


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth_hold_action_server");

  DepthHoldAction depthhold("depth_hold_action_server");
  ros::spin();

  return 0;
}