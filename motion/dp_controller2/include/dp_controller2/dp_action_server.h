
/*   Written by Kevin and Anders
     Copyright (c) 2019 Beluga AUV, Vortex NTNU.
     All rights reserved. */

/**
 * @file
 * @brief A ROS wrapper layer for the DP server
 *
 */

// #include <map>
// #include <math.h>
// #include <string>
// #include <vector>


// #include <actionlib/server/simple_action_server.h>
// //#include <dynamic_reconfigure/server.h>
// #include <geometry_msgs/Pose.h>
// #include <geometry_msgs/PoseArray.h>
// //#include <move_base_msgs/MoveBaseAction.h>
// #include <nav_msgs/Odometry.h>
// #include <ros/ros.h>
// #include <std_msgs/String.h>
// #include <tf/transform_datatypes.h>

// #include "eigen_typedefs.h"

// #include "dp_controller2/quaternion_dp_controller.h"



/**
 * @brief the Controller class
 *
 * This class serves as a wrapper for the lower-level controller implementation
 * @see quaternion_pd_controller.h
 *
 */
// class FibonacciAction {
// private:

//   /**
//   * @brief Desired pose in quaternions.
//   */
//   Eigen::Vector7d eta_d;
//   Eigen::Vector7d eta_dot_d;

//   Eigen::Vector3d eta_d_pos;
//   Eigen::Quaterniond eta_d_ori;

//   ros::NodeHandle m_nh; /** Nodehandle          */

//   ros::Subscriber m_odometry_sub;    /** Odometry subscriber    */

//   ros::Publisher m_wrench_pub; /** Wrench publisher    */

//   ros::Subscriber m_desiredpoint_sub; /* Subscriber for listening to (the guidance node ....)      */
//   ros::Publisher m_referencepoint_pub; /* Publisher for the DP-controller */

//   // EIGEN CONVERSION INITIALIZE
//   Eigen::Vector3d position;       /** Current position      */
//   Eigen::Quaterniond orientation; /** Current orientation   */
//   Eigen::Vector6d velocity;       /** Current velocity      */


//   QuaternionPIDController m_controller;
  
//   protected:
//   // private:

//   ros::NodeHandle nh_;
//   actionlib::SimpleActionServer<dp_controller2::dpAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
//   std::string action_name_;
//   // create messages that are used to published feedback/result
//   dp_controller2::dpFeedback feedback_;
//   dp_controller2::dpResult result_;

// public:


  // /**
  //  * @brief Controller class constructor
  //  *
  //  * @param nh ROS nodehandle
  //  */
  // explicit Controller(ros::NodeHandle nh);

  // /**
  //  * @brief Callback for the odometry subscriber
  //  *
  //  * @param msg   A nav_msg::Odometry message containing state data about the
  //  * AUV.
  //  */


  // void odometryCallback(const nav_msgs::Odometry &msg);
  // void desiredPointCallback(const geometry_msgs::PoseArray &desired_msg);
  // Eigen::Quaterniond EulerToQuaternion(double roll, double pitch, double yaw);
  // Eigen::Vector3d QuaterniondToEuler(Eigen::Quaterniond q);
  // void spin();


  // FibonacciAction(std::string name);

  // ~FibonacciAction(void){};

  // void executeCB(const dp_controller2::dpGoalConstPtr &goal);


// };

//---------------------------------------------------

#ifndef VORTEX_DP_SERVER_H
#define VORTEX_DP_SERVER_H


#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <dp_controller2/dpAction.h>

#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include "eigen_typedefs.h"


class DpAction {
  private:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<dp_controller2::dpAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    std::string action_name_;
    // create messages that are used to published feedback/result
    dp_controller2::dpFeedback feedback_;
    dp_controller2::dpResult result_;

  public:
    DpAction(std::string name);

    ~DpAction(void){};

    void executeCB(const dp_controller2::dpGoalConstPtr &goal);
    
    Eigen::Vector6d pose;

    dp_controller2::dpGoal goal_;

  };

#endif // VORTEX_DP_SERVER_H
