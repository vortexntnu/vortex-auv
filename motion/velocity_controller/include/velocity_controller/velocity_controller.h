#ifndef VELOCITY_CONTROLLER_H
#define VELOCITY_CONTROLLER_H

#include <string>
#include <vector>
#include <memory> // for std::make_unique

#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Wrench.h>
#include <std_srvs/Empty.h>

#include <eigen3/Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

#include "MiniPID.h"
#include "vortex_msgs/SetPidGains.h"

// These typdefs are lacking from the default eigen namespace
namespace Eigen
{
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
}  // namespace Eigen

/**
 * @brief class of a velocity controller that uses six one dimensional PID controllers
 * with feed-forward term and integral windup protection. The control law includes
 * compensation for restoring forces.
 *
 */
class VelocityController
{
public:
  /**
   * @brief Construct a new Velocity Controller object
   *
   * @param nh ros node handle
   */
  VelocityController(ros::NodeHandle nh);

private:
  /**
   * @brief updates local copy of velocity as eigen vector and orientation as quaternion
   *
   * @param odom_msg message with odometry data
   */
  void odometryCallback(const nav_msgs::Odometry& odom_msg);

  /**
   * @brief publishes a thrust given by a control law
   *
   * @param twist_msg message with desired velocity
   */
  void controlLawCallback(const geometry_msgs::Twist& twist_msg);

  /**
   * @brief resets all six PIDs. Clears I and D term and sets setpoint to current position.
   *
   * @param request empty
   * @param response empty
   * @return true if reset did not crash
   */
  bool resetPidCallback(std_srvs::EmptyRequest& request, std_srvs::EmptyResponse& response);

  /**
   * @brief Sets new P, I, D, F and windup terms for all six PIDs. Does a reset of PIDs.
   *
   * @param request new gains that PIDs should be set to
   * @param response empty
   * @return true if operation did not crash
   */
  bool setGainsCallback(vortex_msgs::SetPidGainsRequest& request, vortex_msgs::SetPidGainsResponse& response);

  /**
   * @brief local wrapper around ros::getParam(). Shuts down node if param is not found.
   *
   * @tparam T string, double, float, int bool or vectors of these
   * @param name name of the param on the ros network
   * @param variable variable the param will be saved in
   */
  template <typename T>
  void getParam(std::string name, T& variable);

  /**
   * @brief local wrapper around ros::getParam(). Sets variable to default value if param is not found.
   *
   * @tparam T string, double, float, int bool or vectors of these
   * @param name name of the param on the ros network
   * @param variable variable the param will be saved in
   * @param default_value default value that variable will be set to
   */
  template <typename T>
  void getParam(std::string name, T& variable, T& default_value);

  /**
   * @brief calculates resotring forces acting on drone caused by buoyancy and gravity
   *
   * @return Eigen::Vector6d vector containing restoring forces
   */
  Eigen::Vector6d restoringForces();

  ros::NodeHandle nh;
  std::string odometry_topic;
  std::string thrust_topic;
  std::string desired_velocity_topic;
  double drone_weight;
  float drone_buoyancy;
  Eigen::Vector3d center_of_gravity;
  Eigen::Vector3d center_of_buoyancy;
  ros::Publisher thrust_pub;
  ros::Subscriber odom_sub;
  ros::Subscriber vel_sub;
  ros::ServiceServer reset_service;
  ros::ServiceServer set_gains_service;
  Eigen::Vector6d velocity;
  Eigen::Quaterniond orientation;
  std::vector<std::unique_ptr<MiniPID>> pids;
  bool odom_recieved;
  bool use_restoring_forces;
};
#endif
