#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib_tutorials/FibonacciAction.h>

using namespace actionlib_tutorials;
typedef actionlib::SimpleActionClient<FibonacciAction> Client;

class FibonacciClient{
  private:
    Client ac;



  public:

    void doneCallback(const actionlib::SimpleClientGoalState& state,
                      const FibonacciResultConstPtr& result){
    
      ROS_INFO("Finished in state [%s]", state.toString().c_str());
      ROS_INFO("Answer: %i", result->sequence.back());
      ros::shutdown();
  }


      FibonacciClient(): ac("fibonacci", true)
    {
      ROS_INFO("Waiting for action server to start.");
      ac.waitForServer();
      ROS_INFO("Action server started, sending goal.");  
    }

  void send_goal(int order)
  {
    FibonacciGoal goal;
    goal.order = order;

    // Need boost::bind to pass in the 'this' pointer
    ac.sendGoal(goal,
                boost::bind(&FibonacciClient::doneCallback, this, _1, _2),
                boost::bind(&FibonacciClient::activeCallback, this),
                boost::bind(&FibonacciClient::feedbackCallback, this, _1));

  }

  void feedbackCallback(const FibonacciFeedbackConstPtr& feedback)
{
  //ROS_INFO("Got Feedback of length %lu", feedback->sequence.size());
  ROS_INFO("Got Feedback of length %lu", feedback->sequence.back());

}


  void activeCallback(){
    ROS_INFO("Goal just went active");
  }

};



int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_fibonacci");
  FibonacciClient my_Client;
  my_Client.send_goal(20);

  ros::Rate r(0.2);
  r.sleep();
  my_Client.send_goal(10);

  
  ros::spin();
  return 0;
}