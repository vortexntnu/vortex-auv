#include "dp_reference_model2/reference_model.h"
#include <iostream>
#include <eigen3/Eigen/Dense>
#include "dp_reference_model2/eigen_typedefs.h"

ReferenceModel::ReferenceModel(ros::NodeHandle nh) {
  if (!nh.getParam("dp_rm/zeta_1", zeta_1)) {
    zeta_1 = 1.5;
  }
  if (!nh.getParam("dp_rm/zeta_1", zeta_2)) {
    zeta_2 = 1.5;
  }
  if (!nh.getParam("dp_rm/zeta_1", zeta_3)) {
    zeta_3 = 1.5;
  }
  if (!nh.getParam("dp_rm/zeta_1", zeta_4)) {
    zeta_4 = 1;
  }
  if (!nh.getParam("dp_rm/zeta_1", zeta_5)) {
    zeta_5 = 1;
  }
  if (!nh.getParam("dp_rm/zeta_1", zeta_6)) {
    zeta_6 = 1;
  }
  if (!nh.getParam("dp_rm/zeta_1", zeta_7)) {
    zeta_7 = 1;
  }
  if (!nh.getParam("dp_rm/zeta_1", omega_1)) {
    omega_1 = 0.4;
  }
  if (!nh.getParam("dp_rm/zeta_1", omega_2)) {
    omega_2 = 0.4;
  }
  if (!nh.getParam("dp_rm/zeta_1", omega_3)) {
    omega_3 = 0.4;
  }
  if (!nh.getParam("dp_rm/zeta_1", omega_4)) {
    omega_4 = 1;
  }
  if (!nh.getParam("dp_rm/zeta_1", omega_5)) {
    omega_5 = 1;
  }
  if (!nh.getParam("dp_rm/zeta_1", omega_6)) {
    omega_6 = 1;
  }
  if (!nh.getParam("dp_rm/zeta_1", omega_7)) {
    omega_7 = 1;
  }

    Delta = Eigen::Matrix7d::Zero();
    Omega = Eigen::Matrix7d::Zero();
    Delta.diagonal() << zeta_1, zeta_2, zeta_3, zeta_4, zeta_5, zeta_6, zeta_7;
    // std::cout << "Delta:" << std::endl << Delta << std::endl;
    Omega.diagonal() << omega_1, omega_2, omega_3, omega_4, omega_5, omega_6, omega_7;
    // std::cout << "Omega:" << std::endl << Omega << std::endl;
  
    A_d = Eigen::MatrixXd::Zero(14,14);
    A_d << Eigen::Matrix7d::Zero(), Eigen::Matrix7d::Identity(), -Omega*Omega, -2*Delta*Omega;
    // std::cout << "Omega*Omega:" << std::endl << -Omega*Omega << std::endl;
    // std::cout << "-2*Delta*Omega:" << std::endl << -2*Delta*Omega << std::endl;
    B_d = Eigen::MatrixXd::Zero(14,7);
    B_d << Eigen::Matrix7d::Zero(), Omega*Omega;
    // std::cout << "A_d:" << std::endl << A_d<< std::endl;
    // std::cout << "B_d:" << std::endl << B_d << std::endl;

  eta_d = Eigen::Vector7d::Zero();
  eta_d(3) = 1;
  eta_dot_d = Eigen::Vector7d::Zero();

   // subs and pubs
  setpoint_sub =
      nh.subscribe("/dp_data/reference_point", 1, &ReferenceModel::setpointCallback, this);
  reference_pub =
      nh.advertise<geometry_msgs::PoseArray>("/reference_model/output", 1, this);
    
}
//-----------------------------------------//


Eigen::Quaterniond EulerToQuaternion(double roll, double pitch, double yaw){  
  Eigen::Quaterniond q;
  q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
  //std::cout << "Quaternion" << std::endl << q.coeffs() << std::endl;
  return q;
}



//Eigen::Vector14d 
void ReferenceModel::calculate_smooth(Eigen::Vector7d x_ref){
    Eigen::Vector14d x_d;
    x_d << eta_d, eta_dot_d;
    // std::cout << "x_d1:" << std::endl << x_d << std::endl;

    Eigen::Vector14d x_dot_d = A_d * x_d + B_d * x_ref;
    std::cout << "x_dot_d:" <<std::endl << x_dot_d << std::endl;
    x_d = x_d + time_step * x_dot_d;
    // std::cout << "x_d2:" << std::endl << x_d << std::endl;
    eta_d = x_d.segment(0,7);

    eta_dot_d = x_d.segment(7,7);
    Eigen::VectorXd saturation_vec = Eigen::VectorXd::Zero(7);
    saturation_vec << 1.0, 1.0, 1.0, 1.0/6.0, 1.0/6.0, 1.0/6.0, 1.0/6.0;
    double vel_sat = 1.0;
    for (int i = 0; i < 7; i++){
      eta_dot_d(i) = std::max(std::min(eta_dot_d(i), saturation_vec(i)), -saturation_vec(i));
    }

    // Normalizing desired quaternion
    Eigen::Quaterniond quat_d(eta_d(3), eta_d(4), eta_d(5), eta_d(6));
    quat_d.normalize();
    Eigen::Vector4d quat_d_vec(quat_d.w(), quat_d.x(), quat_d.y(), quat_d.z());
    eta_d.segment(3, 4) = quat_d_vec;

    
    // ------------- //
    // - Change name from calculate smooth


}


// void ReferenceModel::spin() {
//   ros::Rate rate(1);
//   Eigen::Vector3d position_setpoint(1,0,0);
//   Eigen::Quaterniond orientation_setpoint = EulerToQuaternion(0,0,0);

//   Eigen::Vector3d position_test(0,0,0);
//   Eigen::Quaterniond orientation_test = EulerToQuaternion(0,0,0);
// //   Eigen::Vector6d velocity_test = Eigen::Vector6d::Zero();

// Eigen::Vector7d x_ref;
// x_ref << position_setpoint, orientation_setpoint.w(), orientation_setpoint.vec();
// std::cout << x_ref << std::endl;
//   while (ros::ok()) {
//     std::cout << std::endl << "LOOP:" << std::endl;
//     // calculate_smooth(x_ref);
//     // std::cout << "eta_d: " << std::endl << eta_d << std::endl;
//     // std::cout << "eta_dot_d" << std::endl << eta_dot_d << std::endl;
//     ros::spinOnce();
//     rate.sleep();
//   }
// }

void ReferenceModel::setpointCallback(const geometry_msgs::Pose &setpoint_msg) {

  // parse msg
  Eigen::Vector7d x_ref = Eigen::Vector7d::Zero();
  x_ref << setpoint_msg.position.x, setpoint_msg.position.y, setpoint_msg.position.z,
          setpoint_msg.orientation.w, setpoint_msg.orientation.x, setpoint_msg.orientation.y, setpoint_msg.orientation.z;


  // calculate smooth setpoint
  calculate_smooth(x_ref);

  // convert and publish smooth setpoint
  geometry_msgs::Pose x_d_pose;
  geometry_msgs::Pose x_d_velocity;
  x_d_pose.position.x = eta_d(0);
  x_d_pose.position.y = eta_d(1);
  x_d_pose.position.z = eta_d(2);
  x_d_pose.orientation.w = eta_d(3);
  x_d_pose.orientation.x = eta_d(4);
  x_d_pose.orientation.y = eta_d(5);
  x_d_pose.orientation.z = eta_d(6);

  x_d_velocity.position.x = eta_dot_d(0);
  x_d_velocity.position.y = eta_dot_d(1);
  x_d_velocity.position.z = eta_dot_d(2);
  x_d_velocity.orientation.w = eta_dot_d(3);
  x_d_velocity.orientation.x = eta_dot_d(4);
  x_d_velocity.orientation.y = eta_dot_d(5);
  x_d_velocity.orientation.z = eta_dot_d(6);

  geometry_msgs::PoseArray  posearray;
  posearray.header.stamp = ros::Time::now(); // timestamp of creation of the msg
  posearray.header.frame_id = "ReferenceFrame"; // frame id in which the array is published

  // push in array (in C++ a vector, in python a list)
  posearray.poses.push_back(x_d_pose);
  posearray.poses.push_back(x_d_velocity);


  reference_pub.publish(posearray);
}
