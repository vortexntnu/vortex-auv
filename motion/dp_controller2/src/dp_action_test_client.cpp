#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <dp_controller2/dpAction.h>


#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>


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
    ros::NodeHandle m_nh; /** Nodehandle          */

  public:

    void spin(){
       ros::Rate rate(1);
       std::vector<double> goal_position_vec, goal_orientation_vec;
       std::vector<double> goal_position_vec_buff, goal_orientation_vec_buff;

      while (ros::ok()) {

        if (!m_nh.getParam("/setpoint/position", goal_position_vec)) {
          ROS_FATAL("Failed to read parameter setpoint/position. Shutting down node..");
          ros::shutdown();
        }
        if (!m_nh.getParam("/setpoint/orientation", goal_orientation_vec)) {
          ROS_FATAL("Failed to read parameter setpoint/orientation. Shutting down node..");
          ros::shutdown();
        }
        // m_nh.getParam("/setpoint/position", goal_position_vec);
        // m_nh.getParam("/setpoint/orientation", goal_orientation_vec);
        
        if(goal_position_vec != goal_position_vec_buff || goal_orientation_vec != goal_orientation_vec_buff){
          goal_position_vec_buff = goal_position_vec;
          goal_orientation_vec_buff = goal_orientation_vec;
          Eigen::Vector3d goal_postion = Eigen::Vector3d(goal_position_vec[0], goal_position_vec[1], goal_position_vec[2]);
          Eigen::Vector3d goal_orientation = Eigen::Vector3d(goal_orientation_vec[0], goal_orientation_vec[1], goal_orientation_vec[2]);

          send_goal(goal_postion, goal_orientation);

          std::cout << "TESSSST3" << std::endl;

        }



        std::cout << "TESSSST2" << std::endl;
        ros::spinOnce();
        rate.sleep();

      }
   
    }

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

  void send_goal(Eigen::Vector3d goal_position, Eigen::Vector3d goal_orientation)
  {
    dpGoal goal_;
    // Eigen::Vector3d x_ref_pos(2,0,0);
    Eigen::Quaterniond goal_quad = EulerToQuaterniond(goal_orientation(0),goal_orientation(1), goal_orientation(2));
    tf::pointEigenToMsg(goal_position, goal_.x_ref.position);
    std::cout << std::endl << "et eller annet piss 2" << std::endl;
    std::cout << std::endl << goal_.x_ref.position << std::endl;
    tf::quaternionEigenToMsg(goal_quad, goal_.x_ref.orientation);
    std::cout << std::endl << "goal_position:" << goal_position <<std::endl;

    // std::cout << std::endl << "----------------_DOF!_------------" << std::endl;
    //Desired DOF
    Eigen::VectorXd DOF_desired(6,1);
    DOF_desired << 1,1,0,0,0,0;

    for(int i = 0; i < 6; i++){
      goal_.DOF.push_back(DOF_desired(i));
      std::cout << DOF_desired(i) << std::endl;
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


  // DpActionClient(ros::NodeHandle nh) : m_nh(nh) {}

};


int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_DpActionClient");
  ros::NodeHandle nh;
  DpActionClient my_Client;
  // my_Client.send_goal();

  // ros::Rate r(0.2);
  // r.sleep();
  // my_Client.send_goal(10);

  my_Client.spin();
  return 0;
}