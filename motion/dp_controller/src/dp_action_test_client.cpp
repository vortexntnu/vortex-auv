/*   Written by Kevin Strandenes and Anders Slåkvik, Student
     Documentation written by Kevin Strandenes and Anders Slåkvik
     Copyright (c) 2023 Beluga AUV, Vortex NTNU.
     All rights reserved. */

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <vortex_msgs/dpAction.h>
#include <ros/ros.h>
#include <vortex_msgs/dpAction.h>

#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

typedef actionlib::SimpleActionClient<vortex_msgs::dpAction> Client;

// Euler To Quaternion
Eigen::Quaterniond EulerToQuaterniond(double roll, double pitch, double yaw) {
  Eigen::Quaterniond q;
  q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
  std::cout << "Quaternion" << std::endl << q.coeffs() << std::endl;
  return q;
}

class DpActionClient {
private:
  Client ac_;
  ros::NodeHandle m_nh; /** Nodehandle          */

public:

  void getParameters(std::string param_name, std::vector<double> &param_variable){
     if (!m_nh.getParam(param_name, param_variable)) {
        ROS_FATAL("Failed to read parameter %s.  Shutting down node..", param_name.c_str());
        ros::shutdown();
      }
  }

  void spin() {
    ros::Rate rate(1);
    std::vector<double> goal_position_vec, goal_orientation_vec, goal_DOF_vec;
    std::vector<double> goal_position_vec_buff, goal_orientation_vec_buff,
        goal_DOF_vec_buff;
    while (ros::ok()) {

      //get ROS parameters
      getParameters("/setpoint/position", goal_position_vec);
      getParameters("/setpoint/orientation", goal_orientation_vec);
      getParameters("/setpoint/DOF", goal_DOF_vec);
      
      actionlib::SimpleClientGoalState state = ac_.getState();
      ROS_INFO("Action finished: %s", state.toString().c_str());

      // Check if goal has has changed
      if (goal_position_vec != goal_position_vec_buff ||
          goal_orientation_vec != goal_orientation_vec_buff ||
          goal_DOF_vec != goal_DOF_vec_buff) {
        goal_position_vec_buff = goal_position_vec;
        goal_orientation_vec_buff = goal_orientation_vec;
        goal_DOF_vec_buff = goal_DOF_vec;
        Eigen::Vector3d goal_postion = Eigen::Vector3d(
            goal_position_vec[0], goal_position_vec[1], goal_position_vec[2]);
        Eigen::Vector3d goal_orientation =
            Eigen::Vector3d(goal_orientation_vec[0], goal_orientation_vec[1],
                            goal_orientation_vec[2]);
        Eigen::VectorXd goal_DOF = Eigen::VectorXd::Zero(6);
        goal_DOF << goal_DOF_vec[0], goal_DOF_vec[1], goal_DOF_vec[2],
            goal_DOF_vec[3], goal_DOF_vec[4], goal_DOF_vec[5];

        //sending new goal to action server
        send_goal(goal_postion, goal_orientation, goal_DOF);
      }

      ros::spinOnce();
      rate.sleep();
    }
  }

  //reached desired goal
  void doneCallback(const actionlib::SimpleClientGoalState &state,
                    const vortex_msgs::dpResultConstPtr &result) {

    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    ROS_INFO("Answer: %i", result->finished);
  }

  DpActionClient() : ac_("DpAction", true) {
    ROS_INFO("Waiting for action server to start.");
    ac_.waitForServer();
    ROS_INFO("Action server started, sending goal.");
  }

  void send_goal(Eigen::Vector3d goal_position,
                 Eigen::Vector3d goal_orientation, Eigen::VectorXd goal_DOF) {
    vortex_msgs::dpGoal goal_;
    Eigen::Quaterniond goal_quad = EulerToQuaterniond(
        goal_orientation(0), goal_orientation(1), goal_orientation(2));
    tf::pointEigenToMsg(goal_position, goal_.x_ref.position);
    tf::quaternionEigenToMsg(goal_quad, goal_.x_ref.orientation);

    for (int i = 0; i < 6; i++) {
      goal_.DOF.push_back(goal_DOF(i));
    }

    ac_.sendGoal(goal_,
                 boost::bind(&DpActionClient::doneCallback, this, _1, _2),
                 boost::bind(&DpActionClient::activeCallback, this),
                 boost::bind(&DpActionClient::feedbackCallback, this, _1));
  }

  void feedbackCallback(const vortex_msgs::dpFeedbackConstPtr &feedback) {
    ROS_INFO("Got Feedback %f %f %f %f %f %f", feedback->error[0],
             feedback->error[1], feedback->error[2], feedback->error[3],
             feedback->error[4], feedback->error[5]);
  }

  void activeCallback() { ROS_INFO("Goal just went active"); }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_DpActionClient");
  ros::NodeHandle nh;
  DpActionClient my_Client;

  my_Client.spin();
  return 0;
}


