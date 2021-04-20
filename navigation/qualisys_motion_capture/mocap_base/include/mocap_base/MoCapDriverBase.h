/*
 * Copyright [2015] [Ke Sun <sunke.polyu@gmail.com>]
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef MOCAP_DRIVER_BASE_H
#define MOCAP_DRIVER_BASE_H


#include <map>
#include <string>
#include <vector>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <mocap_base/KalmanFilter.h>

namespace mocap{

/*
 * @brief Subject Defines attributes of a rigid body
 */
class Subject {
  public:
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
     
    enum Status {
      LOST,
      INITIALIZING,
      TRACKED
    };

    /*
     * @brief Constructor and Destructor
     */
    Subject(ros::NodeHandle* nptr, const std::string& sub_name,
        const std::string& p_frame);
    ~Subject() {}

    /*
     * @brief getName setName
     *    Get and set the name of the object
     */
    const std::string& getName();
    void setName(const std::string& sub_name);
    /*
     * @brief isActive Tells if the object is still active or not
     */
    const Status& getStatus();
    void enable();
    void disable();
    /*
     * @brief getAttitude getPosition getAngularVel getLinearVel
     *    Returns the state of the object
     */
    const Eigen::Quaterniond& getAttitude();
    const Eigen::Vector3d& getPosition();
    const Eigen::Vector3d& getAngularVel();
    const Eigen::Vector3d& getLinearVel();

    /*
     * @brief setNoiseParameter Set noise parameters for
     *    the kalman filter
     * @param u_cov Input noise
     * @param m_cov Measurement noise
     * @return True if success
     */
    bool setParameters(
        const Eigen::Matrix<double, 12, 12>& u_cov,
        const Eigen::Matrix<double, 6, 6>& m_cov,
        const int& freq);

    /*
     * @brief processNewMeasurement Process new measurements
     *    from the mocap system
     * @param m_attitude Measured attitude
     * @param m_position Measured position
     */
    void processNewMeasurement(
        const double& time,
        const Eigen::Quaterniond& m_attitude,
        const Eigen::Vector3d& m_position);


    typedef boost::shared_ptr<Subject> SubjectPtr;
    typedef const boost::shared_ptr<Subject> SubjectConstPtr;

  private:
    // Disable copy constructor and assign operator
    Subject(const Subject&);
    Subject& operator=(const Subject&);

    // Name of the subject
    std::string name;

    // Error state Kalman filter
    KalmanFilter kFilter;

    // Tells the status of the object
    Status status;

    // Prevent cocurrent reading and writing of the class
    boost::shared_mutex mtx;

    // Publisher for the subject
    ros::NodeHandle* nh_ptr;
    std::string parent_frame;
    ros::Publisher pub_filter;
    ros::Publisher pub_raw;
    ros::Publisher pub_vel;
};

/*
 * @brief: MoCapDriverBase Base class for the drivers
 *    of different motion capture system (e.g. vicon
 *    and qualisys)
 */
class MoCapDriverBase{

  public:
    /*
     * @brief Constructor
     * @param nh Ros node
     */
    MoCapDriverBase(const ros::NodeHandle& n):
      nh             (n),
      frame_rate     (100),
      model_list     (std::vector<std::string>(0)),
      publish_tf     (false),
      fixed_frame_id ("mocap"){
      return;
    }

    /*
     * @brief Destructor
     */
    ~MoCapDriverBase() {
      return;
    }

    /*
     * @brief init Initialize the object
     * @return True if successfully initialized
     */
    virtual bool init() = 0;

    /*
     * @brief run Start acquiring data from the server once
     */
    virtual bool run() = 0;

    /*
     * @brief disconnect Disconnect to the server
     * @Note The function is called automatically when the
     *  destructor is called.
     */
    virtual void disconnect() = 0;

  private:
    // Disable the copy constructor and assign operator
    MoCapDriverBase(const MoCapDriverBase& );
    MoCapDriverBase& operator=(const MoCapDriverBase& );

  protected:
    /*
     * @brief handleFrame handles a whole data package from
     *  the motion capture system
     */
    virtual void handleFrame() = 0;

    /*
     * @brief handSubject Handles a single subject (rigid body)
     *  from the motion capture system
     * @param sub_idx Index of the subject to be handled
     */
    virtual void handleSubject(int sub_idx) = 0;

    // Address of the server
    std::string server_address;

    // Ros node
    ros::NodeHandle nh;

    // Frame rate of the mocap system
    unsigned int frame_rate;

    // Rigid body to be tracked
    // Empty string if all the objects in the arena are to be tracked
    std::vector<std::string> model_list;

    // Observed rigid bodies (contains those lose tracking)
    std::map<std::string, Subject::SubjectPtr> subjects;

    // Publish tf
    bool publish_tf;
    std::string fixed_frame_id;
    tf::TransformBroadcaster tf_publisher;

};
}


#endif
