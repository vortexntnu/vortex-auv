/**
 * @file
 * 
 * Written by Jae Hyeong Hwang
 * Modified by Øystein Solbø
 * Copyright (c) 2021 Manta AUV, Vortex NTNU.
 * All rights reserved.
 * 
 * 
 * Module connecting the DP-controller to the joystick_guidance and 
 * the dp_guidance
 */
#ifndef DP_REFERENCE_MODEL_H
#define DP_REFERENCE_MODEL_H


#include <Eigen/Dense>
#include <math.h>

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "eigen_conversions/eigen_msg.h"
#include "tf/tf.h"
#include "tf_conversions/tf_eigen.h"


using namespace Eigen;

class ReferenceModel 
{   
private:
     /* Bool to choose between input from the joystick and the FSM. Can only be set via joystick */
     bool ROV_state;

     /* Eigen-vectors used during calculate_smooth */
     Eigen::Vector3d a_x; 
     Eigen::Vector3d b_x;

     /* Positions */
     Eigen::Vector3d x_d_prev;         /** Previous desired body position            */
     Eigen::Vector3d x_d_prev_prev;    /** Previous previous desired body position   */
     Eigen::Vector3d x_ref_prev;       /** Previous reference body position          */
     Eigen::Vector3d x_ref_prev_prev;  /** Previous previous reference body position */

     /**
      * @brief Callback-function calculating desired Pose
      * when joystick is used. 
      * 
      * @warning Only published to DP-controller if ROV_state == true 
      * 
      * @param msg Reference Pose
      */
     void joystick_setpoint_cb(const geometry_msgs::Pose& msg);


     /**
      * @brief Callback-function calculating desired Pose when topic from
      * guidance_block (FSM) is updated
      * 
      * @warning Only published further if ROV_state == false
      * 
      * @param msg Reference Pose
      */
     void fsm_setpoint_cb(const geometry_msgs::Pose& msg);


     /**
      * @brief Callback to change the desired state of the UUV
      * 
      * @param msg Bool determining if the system should be set as a ROV or as an AUV
      * msg == false => ROV
      * msg == true  => AUV
      */
     void uuv_state_cb(const std_msgs::Bool &msg);



     /**
      * @brief Utility function that calculates a smooth trajectory from current
      * position to the desired position @p x_ref
      * 
      * @param x_ref Reference used 
      */
     Eigen::Vector3d calculate_smooth(const Eigen::Vector3d &x_ref);

     /**
      * @brief Function that resets the private variables to @p pos
      */
     void reset(Eigen::Vector3d pos);


     EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
     /**
      * @brief Constructor
      * 
      * @param nh ROS nodehandle
      */
     ReferenceModel(ros::NodeHandle nh);

     ros::Subscriber uuv_state_sub;               /* Subscriber for changing uuv-state         */
     ros::Subscriber joystick_setpoint_sub;       /* Subscriber for listening to the joystick  */
     ros::Subscriber fsm_setpoint_sub;            /* Subscriber for listening to the FSM       */
     ros::Publisher reference_pub;                /* Publish of point to DP-controller         */

};

#endif  // DP_REFERENCE_MODEL_H