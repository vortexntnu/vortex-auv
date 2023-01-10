#include "dp_reference_model2/reference_model.h"
#include <math.h>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen_typedefs.h>

ReferenceModel::ReferenceModel(ros::NodeHandle nh) {

    Delta.diagonal();
    Omega.diagonal();
    Delta << zeta_1, zeta_2, zeta_3, zeta_4, zeta_5, zeta_6;
    Omega << omega_1, omega_2, omega_3, omega_4, omega_5, omega_6;

    A_d << Eigen::Matrix2d::Zero(), Eigen::Matrix2d::Identity(), -Omega*Omega, -2*Delta*Omega;
    
    A_d << Eigen::Matrix2d::Zero(), Omega*Omega;

    
}
//-----------------------------------------//


Eigen::Quaterniond EulerToQuaternion(double roll, double pitch, double yaw){  
  Eigen::Quaterniond q;
  q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
  std::cout << "Quaternion" << std::endl << q.coeffs() << std::endl;
  return q;
}

//Eigen::Vector14d 
void ReferenceModel::calculate_smooth(Eigen::Vector7d x_ref){
    Eigen::Vector14d x_d;
    x_d << Eta_d, Eta_dot_d;

    Eigen::Vector14d x_dot_d = A_d * x_d + B_d * x_ref;

    x_d = x_d + time_step * x_dot_d;
}


void ReferenceModel::spin() {
  ros::Rate rate(1);
  Eigen::Vector3d position_setpoint(1,0,0);
  Eigen::Quaterniond orientation_setpoint = EulerToQuaternion(0,0,0);

  Eigen::Vector3d position_test(0,0,0);
  Eigen::Quaterniond orientation_test = EulerToQuaternion(0,0,0);
//   Eigen::Vector6d velocity_test = Eigen::Vector6d::Zero();

Eigen::Vector7d x_ref;
x_ref << position_setpoint, orientation_setpoint.w(), orientation_setpoint.vec();
std::cout << x_ref << std::endl;
  while (ros::ok()) {

    std::cout << "TESSSST" << std::endl;
    ros::spinOnce();
    rate.sleep();
  }
}