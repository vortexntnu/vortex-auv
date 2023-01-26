#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <dp_controller2/dpAction.h>


#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
// #include "eigen_typedefs.h"

// using namespace dp_controller2;
// typedef actionlib::SimpleActionClient<dpAction> Client;

// class FibonacciClient{
//   private:
//     Client ac;



//   public:

//     void doneCallback(const actionlib::SimpleClientGoalState& state,
//                       const dpResultConstPtr& result){
    
//       ROS_INFO("Finished in state [%s]", state.toString().c_str());
//       ROS_INFO("Answer: %i", result->sequence.back());
//       ros::shutdown();
//   }


//       FibonacciClient(): ac("fibonacci", true)
//     {
//       ROS_INFO("Waiting for action server to start.");
//       ac.waitForServer();
//       ROS_INFO("Action server started, sending goal.");  
//     }

//   void send_goal(int order)
//   {
//     dpGoal goal;
//     goal.order = order;

//     // Need boost::bind to pass in the 'this' pointer
//     ac.sendGoal(goal,
//                 boost::bind(&FibonacciClient::doneCallback, this, _1, _2),
//                 boost::bind(&FibonacciClient::activeCallback, this),
//                 boost::bind(&FibonacciClient::feedbackCallback, this, _1));

//   }

//   void feedbackCallback(const dpFeedbackConstPtr& feedback)
// {
//   //ROS_INFO("Got Feedback of length %lu", feedback->sequence.size());
//   ROS_INFO("Got Feedback of length %i", feedback->error.back());

// }


//   void activeCallback(){
//     ROS_INFO("Goal just went active");
//   }

// };



// int main (int argc, char **argv)
// {
//   ros::init(argc, argv, "test_fibonacci");
//   FibonacciClient my_Client;
//   my_Client.send_goal(20);

//   ros::Rate r(0.2);
//   r.sleep();
//   my_Client.send_goal(10);

  
//   ros::spin();
//   return 0;
// }



//----------------------------------------------------------

using namespace dp_controller2;
typedef actionlib::SimpleActionClient<dpAction> Client;


//Euler To Quaternion
Eigen::Quaterniond EulerToQuaterniond(double roll, double pitch, double yaw){  
  Eigen::Quaterniond q;
  q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
  std::cout << "Quaternion" << std::endl << q.coeffs() << std::endl;
  return q;
}


class DpActionClient{
  private:
    Client ac_;

  public:

    void doneCallback(const actionlib::SimpleClientGoalState& state,
                      const dpResultConstPtr& result){
    
      ROS_INFO("Finished in state [%s]", state.toString().c_str());
      ROS_INFO("Answer: %i", result->finished);
      ros::shutdown();                // MAYBE REMOVE THIS LINE
    }


      DpActionClient(): ac_("DpAction", true)
    {
      ROS_INFO("Waiting for action server to start.");
      ac_.waitForServer();
      ROS_INFO("Action server started, sending goal.");  
    }

  void send_goal()
  {
    dpGoal goal_;
    Eigen::Vector3d x_ref_pos(2,4,6);
    Eigen::Quaterniond x_ref_ori = EulerToQuaterniond(1,2,3);
    tf::pointEigenToMsg(x_ref_pos, goal_.x_ref.position);
    std::cout << std::endl << "et eller annet piss 2" << std::endl;
    std::cout << std::endl << goal_.x_ref.position << std::endl;
    tf::quaternionEigenToMsg(x_ref_ori, goal_.x_ref.orientation);
    
    // std::cout << std::endl << "----------------_DOF!_------------" << std::endl;
    //Desired DOF
    Eigen::VectorXd DOF_div(6,1);
    DOF_div << 0,0,1,1,0,1;

    for(int i = 0; i < 6; i++){
      goal_.DOF.push_back(DOF_div(i));
      std::cout << DOF_div(i) << std::endl;
    }
    // std::cout << std::endl << "----------------------------" << std::endl;

    // Need boost::bind to pass in the 'this' pointer
    ac_.sendGoal(goal_,
                boost::bind(&DpActionClient::doneCallback, this, _1, _2),
                boost::bind(&DpActionClient::activeCallback, this),
                boost::bind(&DpActionClient::feedbackCallback, this, _1));

  }

  void feedbackCallback(const dpFeedbackConstPtr& feedback)
{
  // ROS_INFO("Got Feedback of length %lu", feedback->sequence.size());
  ROS_INFO("Got Feedback %f %f %f %f %f %f", feedback->error[0], feedback->error[1],  feedback->error[2],  feedback->error[3],  feedback->error[4],  feedback->error[5]);

}


  void activeCallback(){
    ROS_INFO("Goal just went active");
  }

};



int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_DpActionClient");
  DpActionClient my_Client;
  my_Client.send_goal();

  // ros::Rate r(0.2);
  // r.sleep();
  // my_Client.send_goal(10);

  
  ros::spin();
  return 0;
}