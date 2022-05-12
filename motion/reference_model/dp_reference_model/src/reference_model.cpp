/*   Written by Jae Hyeong Hwang and Christopher Strøm
     Modified by Øystein Solbø
     Copyright (c) 2021 Vortex NTNU.
     All rights reserved. */

#include "dp_reference_model/reference_model.h"

ReferenceModel::ReferenceModel(ros::NodeHandle nh) {
  // get params
  double a1, a2, a3;
  double b1, b2, b3;
  std::vector<double> beta_temp;

  if (!nh.getParam("dp_rm/a1", a1))
    a1 = 1;
  if (!nh.getParam("dp_rm/a2", a2))
    a2 = -1.990024937655860;
  if (!nh.getParam("dp_rm/a3", a3))
    a3 = 0.990049813123053;
  if (!nh.getParam("dp_rm/b1", b1))
    b1 = 6.218866798092052e-06;
  if (!nh.getParam("dp_rm/b2", b2))
    b2 = 1.243773359618410e-05;
  if (!nh.getParam("dp_rm/b3", b3))
    b3 = 6.218866798092052e-06;
  if (!nh.getParam("/reference_model/dp/beta", beta_temp))
    beta_temp = {0.025, 0.025, 0.0025};

  a_x = Eigen::Vector3d(a1, a2, a3);
  b_x = Eigen::Vector3d(b1, b2, b3);
  beta = Eigen::Vector3d(beta_temp[0], beta_temp[1], beta_temp[2]);

  // initiate local variables
  prev_control_mode = 0; // OPEN LOOP
  x_d_prev = Eigen::Vector3d::Zero();
  x_d_prev_prev = Eigen::Vector3d::Zero();
  x_ref_prev = Eigen::Vector3d::Zero();
  x_ref_prev_prev = Eigen::Vector3d::Zero();

  // subs and pubs
  setpoint_sub =
      nh.subscribe("/guidance/dp_data", 1, &ReferenceModel::setpoint_cb, this);
  reference_pub =
      nh.advertise<vortex_msgs::DpSetpoint>("/reference_model/output", 1, this);
}

void ReferenceModel::setpoint_cb(const vortex_msgs::DpSetpoint &setpoint_msg) {

  // parse msg
  Eigen::Vector3d x_ref{setpoint_msg.setpoint.position.x,
                        setpoint_msg.setpoint.position.y,
                        setpoint_msg.setpoint.position.z};

  // check if control mode has changed
  if (setpoint_msg.control_mode != prev_control_mode) {
    reset(x_ref); // reset prev values to current target position
    prev_control_mode = setpoint_msg.control_mode;
    ROS_DEBUG("DP reference model reset");
  }

  // calculate smooth setpoint
  Eigen::Vector3d x_d = calculate_smooth(x_ref);

  // convert and publish smooth setpoint
  geometry_msgs::Point x_d_point;
  tf::pointEigenToMsg(x_d, x_d_point);

  vortex_msgs::DpSetpoint dp_setpoint;
  dp_setpoint.setpoint.position = x_d_point;
  dp_setpoint.setpoint.orientation = setpoint_msg.setpoint.orientation;
  dp_setpoint.control_mode = setpoint_msg.control_mode;

  reference_pub.publish(dp_setpoint);
}

Eigen::Vector3d ReferenceModel::low_pass_filter(const Eigen::Vector3d &x_ref) {
  Eigen::Vector3d x_d;
  for (int i = 0; i < 3; i++) {
    x_d[i] = x_d_prev[i] - beta[i] * (x_d_prev[i] - x_ref[i]);
  }
  return x_d;
}

Eigen::Vector3d ReferenceModel::calculate_smooth(const Eigen::Vector3d &x_ref) {
  Eigen::Vector3d x_d;
  x_d = b_x(0) * x_ref + b_x(1) * x_ref_prev + b_x(2) * x_ref_prev_prev -
        a_x(1) * x_d_prev - a_x(2) * x_d_prev_prev;

  x_ref_prev_prev = x_ref_prev;
  x_ref_prev = x_ref;
  x_d_prev_prev = x_d_prev;
  x_d_prev = x_d;

  return x_d;
}

void ReferenceModel::reset(Eigen::Vector3d pos) {
  x_d_prev = pos;
  x_d_prev_prev = pos;
  x_ref_prev = pos;
  x_ref_prev_prev = pos;
}
