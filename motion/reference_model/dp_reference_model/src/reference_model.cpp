/*   Written by Jae Hyeong Hwang and Christopher Strøm
     Modified by Øystein Solbø
     Copyright (c) 2021 Vortex NTNU.
     All rights reserved. */


#include "dp_reference_model/reference_model.h"

ReferenceModel::ReferenceModel(ros::NodeHandle nh) : ROV_state{false}
{
     /**
      * Have used the old values (if changed) as default-values. 
      * If ROS is unable to locate new values, the old ones should be good  
      */
     double a1 = nh.getParam("rm_a1", 1);
     double a2 = nh.getParam("rm_a2", -1.990024937655860);
     double a3 = nh.getParam("rm_a3", 0.990049813123053);

     double b1 = nh.getParam("rm_b1", 6.218866798092052e-06);
     double b2 = nh.getParam("rm_b2", 1.243773359618410e-05);
     double b3 = nh.getParam("rm_b3", 6.218866798092052e-06);

     Eigen::Vector3d a_x(a1, a2, a3);
     Eigen::Vector3d b_x(b1, b2, b3);

     Eigen::Vector3d x_d_prev            = Eigen::Vector3d::Zero();
     Eigen::Vector3d x_d_prev_prev       = Eigen::Vector3d::Zero();
     Eigen::Vector3d x_ref_prev          = Eigen::Vector3d::Zero();
     Eigen::Vector3d x_ref_prev_prev     = Eigen::Vector3d::Zero();

     uuv_state_sub            = nh.subscribe("/guidance/joystick_state", 1, &ReferenceModel::uuv_state_cb, this);
     joystick_setpoint_sub    = nh.subscribe("/guidance/joystick_reference", 1, &ReferenceModel::joystick_setpoint_cb, this);
     fsm_setpoint_sub         = nh.subscribe("/reference_model/input", 1, %ReferenceModel::fsm_setpoint_cb, this); 
     reference_pub            = nh.advertise<geometry_msgs::Pose>("/reference_model/output", 10, this);
}


// Callbacks
void ReferenceModel::joystick_setpoint_cb(const geometry_msgs::Pose &msg) 
{
     /* Only accepting if driving as a ROV */
     if(ROV_state){
          calculate_desired_pose(msg);
     }
}

void ReferenceModel::fsm_setpoint_cb(const geometry_msgs::Pose &msg) 
{
     /* Only accepting if driving as an AUV */
     if(!ROV_state){
          calculate_desired_pose(msg);
     }
}

void ReferenceModel::uuv_state_cb(const std_msgs::Bool &msg)
{
     /* Changing the state between ROV <--> AUV */
     ROV_state = msg.data;
}


// Utility
Eigen::Vector3d ReferenceModel::calculate_smooth(const Eigen::Vector3d &x_ref)
{
     Eigen::Vector3d x_d;
     x_d = b_x(0) * x_ref + b_x(1) * x_ref_prev + b_x(2) * x_ref_prev_prev - a_x(1) * x_d_prev - a_x(2) * x_d_prev_prev;
     
     x_ref_prev_prev = x_ref_prev;
     x_ref_prev = x_ref;
     x_d_prev_prev = x_d_prev;
     x_d_prev = x_d;

     return x_d;
}


void ReferenceModel::reset(Eigen::Vector3d pos)
{
     x_d_prev = pos;
     x_d_prev_prev = pos;
     x_ref_prev = pos;
     x_ref_prev_prev = pos;
}


void ReferenceModel::calculate_desired_pose(const geometry_msgs::Pose &msg)
{
     Eigen::Vector3d x_ref{msg.position.x, msg.position.y, msg.position.z};
     Eigen::Vector3d x_d = calculate_smooth(x_ref);

     geometry_msgs::Point x_d_point;
     tf::pointEigenToMsg(x_d, x_d_point);

     geometry_msgs::Pose pose;
     pose.position = x_d_point;
     pose.orientation = msg.orientation;
     reference_pub.publish(pose);
}