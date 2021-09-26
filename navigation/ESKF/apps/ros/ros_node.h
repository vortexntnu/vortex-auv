#pragma once

#include <Eigen/Core>
#include <chrono>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>

#include "ESKF.h"

const Eigen::Matrix3d R_ACC((Eigen::Matrix3d() << 4, 0, 0, 0, 4, 0, 0, 0, 4).finished());
const Eigen::Matrix3d R_ACCBIAS((Eigen::Matrix3d() << 6e-5, 0, 0, 0, 6e-5, 0, 0, 0, 6e-5).finished());
const Eigen::Matrix3d R_GYRO((Eigen::Matrix3d() << 12e-3, 0, 0, 0, 12e-3, 0, 0, 0, 12e-3).finished());
const Eigen::Matrix3d R_GYROBIAS((Eigen::Matrix3d() << 3e-7, 0, 0, 0, 3e-7, 0, 0, 0, 3e-7).finished());
constexpr double P_GYRO_BIAS{ 0.0001 };
constexpr double P_ACC_BIAS{ 0.0001 };
const Eigen::Matrix3d
  S_A((Eigen::Matrix3d() << -0.9990, 1.9804e-04, -0.0450, 1.3553e-20, 1.0, 0.0044, 0.0450, 0.0044, -0.9990).finished());
const Eigen::Matrix3d
  S_G((Eigen::Matrix3d() << -0.9990, 1.9804e-04, -0.0450, 1.3553e-20, 1.0, 0.0044, 0.0450, 0.0044, -0.9990).finished());
const Eigen::Matrix3d S_DVL((Eigen::Matrix3d() << 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0).finished());
const Eigen::Matrix3d S_INC(
  (Eigen::Matrix3d() << -0.9990, 1.9804e-04, -0.0450, 1.3553e-20, 1.0, 0.0044, 0.0450, 0.0044, -0.9990).finished());
const Eigen::Matrix3d R_DVL((Eigen::Matrix3d() << 1.0e-10, 0, 0, 0, 1.0e-10, 0, 0, 0, 1.0e-10).finished());
const Eigen::MatrixXd R_PRESSUREZ((Eigen::MatrixXd(1, 1) << 2.2500).finished());

const Eigen::Vector3d IMURPYTONED((Eigen::Vector3d() << 0.0, M_PI, 0.0).finished());
const Eigen::Vector3d IMUALIGNMENT((Eigen::Vector3d() << -0.004635, 0.009897, 0).finished());  // Roll -0.004397 , pitch
                                                                                               // 0.0450556 , yaw

const Eigen::Vector3d roll_pitch_yaw_NED_and_alignment_corrected = IMURPYTONED + IMUALIGNMENT;

class ESKF_Node
{
public:
  ESKF_Node(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  //~ESKF_Node();

  // void getParametersFromYamlFile();

private:
  ros::NodeHandle nh_;
  bool init_;

  Eigen::Matrix3d R_dvl_;
  Eigen::Matrix<double, 1, 1> R_pressureZ_;

  eskf::ESKF eskf_;

  double TAC_z;

  // Load from Yaml file
  eskf::parametersInESKF loadParametersFromYamlFile();

  // ROS subscribers
  ros::Subscriber subcribeDVL_;
  ros::Subscriber subscribeIMU_;
  ros::Subscriber subscribePressureZ_;
  ros::Timer pubTImer_;

  // Timestamps

  ros::Time previousTimeStampIMU_;
  ros::Time previousTimeStampDVL_;
  ros::Time previousTimeStampPressureZ_;
  double initialIMUTimestamp_;

  // ROS publisher
  ros::Publisher publishPose_;
  ros::Publisher publishAccGyrobiasandGravity_;
  ros::Publisher publish_DVLNIS_;
  ros::Publisher publish_PressureZNIS_;

  // Callbacks
  void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_Message_data);
  void dvlCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& msg);
  void publishPoseState(const ros::TimerEvent&);

  void pressureZCallback(const std_msgs::Float64::ConstPtr& depth_msg);

  // Execution time
  std::vector<double> execution_time_vector_imu_;
  std::vector<double> execution_time_vector_dvl_;
  std::vector<double> execution_time_vector_pressureZ_;
  bool publish_execution_time_dvl_;
  bool publish_execution_time_PressureZ_;
  bool publish_execution_time_imu_;

  // angular velocity
  geometry_msgs::Vector3 angular_vel;
};

double meanOfVector(const std::vector<double>& vec);
double maxOfVector(const std::vector<double>& vec);
double stanardDeviationOfVector(const std::vector<double>& vec);
void setIMUTopicNameFromYaml(std::string& imu_topic_name);
void setDVLTopicNameFromYawl(std::string& dvl_topic_name);
void setPressureZTopicNameFromYaml(std::string& pressure_Z_topic_name);
void setPublishrateFromYaml(int& publish_rate);
void setRdvlFromYamlFile(Eigen::Matrix3d& R_dvl);
void setRpressureZFromYamlFile(Eigen::Matrix<double, 1, 1>& R_pressureZ);
