#ifndef DP_REFERENCE_MODEL_H
#define DP_REFERENCE_MODEL_H

#include "eigen_typedefs.h"

#include <Eigen/Dense>
#include <math.h>

#include "eigen_conversions/eigen_msg.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "ros/ros.h"
#include <dp_reference_model/DpReferenceModelConfig.h> //autogenerated from Controller.cfg
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

// TODO: Add relevant literature or math write up (minimally a link to Vortex
// Wiki).

using namespace Eigen;

class ReferenceModel {
private:
  /**
   * @brief
   */

  Eigen::Vector7d max_vel = Eigen::Vector7d::Zero();

  Eigen::Matrix7d Delta;
  Eigen::Matrix7d Omega;
  Eigen::MatrixXd A_d = Eigen::MatrixXd::Zero(14, 14);
  Eigen::MatrixXd B_d = Eigen::MatrixXd::Zero(14, 7);

  // EIGEN CONVERSION INITIALIZE
  Eigen::Vector3d position;       /** Current position      */
  Eigen::Quaterniond orientation; /** Current orientation   */
  Eigen::Vector6d velocity;       /** Current velocity      */

  /**
   * @brief Desired pose in quaternions.
   */
  Eigen::Vector7d eta_d;
  Eigen::Vector7d eta_dot_d;

  Eigen::Vector7d x_ref = Eigen::Vector7d::Zero();

  // Eigen::Vector14d
  void calculate_smooth(Eigen::Vector7d x_ref);

  ros::NodeHandle m_nh; /** Nodehandle          */

  /**
   * @brief Desires the rate of the reference model.
   */
  double time_step = 0.1;

  template <typename T>
  void getParameters(std::string param_name, T &param_variable);

  void odometryCallback(const nav_msgs::Odometry &msg);
  void cfgCallback(dp_reference_model::DpReferenceModelConfig &config,
                   uint32_t level);
  dynamic_reconfigure::Server<dp_reference_model::DpReferenceModelConfig>
      m_cfg_server;

  ros::Time last_time;

public:
  /**
   * @brief Constructor
   *
   * @param nh ROS nodehandle
   */
  ReferenceModel(ros::NodeHandle nh);

  ros::Subscriber
      setpoint_sub; /* Subscriber for listening to (the guidance node ....) */
  ros::Publisher reference_pub;   /* Publisher for the DP-controller */
  ros::Subscriber m_odometry_sub; // Subscriber for /odometry/filtered

  /**
   * @brief Calculate and publish the desired, smooth position
   * and orientation.
   *
   *
   * @param setpoint_msg target setpoint
   */
  void setpointCallback(const geometry_msgs::Pose &setpoint_msg);

  void spin();
};

#endif