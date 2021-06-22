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

#ifndef MOCAP_KALMAN_FILTER_H
#define MOCAP_KALMAN_FILTER_H

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace mocap{

/*
 * @brief Error state Kalman filter to estimate the pose
 *    of the poses within the arena of the motion capture system
 *
 *    For more details, please refer to
 *      Nikolas Trawny and Stregios I. Roumeliotis, "Indirect Kalman
 *      Filter for 3D Attitude Estimation: A Tutorial for Quaternion
 *      Algebra", Technical Report Number 2005-002, Rev. 57, Mar. 2005
 */
class KalmanFilter {

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef Eigen::Matrix<double,  6,  6> Matrix6d;
    typedef Eigen::Matrix<double, 12, 12> Matrix12d;
    typedef Eigen::Matrix<double,  6, 12> Matrix6_12d;
    typedef Eigen::Matrix<double, 12,  6> Matrix12_6d;
    typedef Eigen::Matrix<double,  6,  1> Vector6d;
    typedef Eigen::Matrix<double, 12,  1> Vector12d;

    // NOTE: These variables are left public for quick access
    //    Do NOT modify them outsize the class

    // State of a rigid body
    Eigen::Quaterniond attitude;    // Takes a vector from inertia frame to body frame
    Eigen::Vector3d position;       // In inertia frame
    Eigen::Vector3d angular_vel;    // In body frame
    Eigen::Vector3d linear_vel;     // In inertia frame

    Matrix12d state_cov;            // State uncertainty
    Matrix12d input_cov;            // Input uncertainty
    Matrix6d  measurement_cov;      // Measurement uncertainty

    /*
     * @brief Constructor and Destructor
     */
    KalmanFilter();
    ~KalmanFilter() {
      return;
    }

    /*
     * @brief init Initialize the kalman filter
     * @param u_cov Input noise
     * @param m_cov Measurement noise
     * @return True if success
     */
    bool init(const Matrix12d& u_cov,
        const Matrix6d& m_cov, const int& freq);

    /*
     * @brief prepareInitialCondition This functions is dedicated
     *    to create the intial condition for the kalman filter. The
     *    function needs to be called twice, once for initializaing
     *    the pose and once for the angular and linear velocities
     * @param m_attitude Measured atttiude from the motion
     *    capture system
     * @param m_position Measured posiiton from the motion
     *    capture system
     * @return True if the input parameters are accepted; False if
     *    the filter has already been intialized
     */
    bool prepareInitialCondition(const double& curr_time_stamp,
        const Eigen::Quaterniond& m_attitude,
        const Eigen::Vector3d& m_position);

    /*
     * @brief isReady Tells if the filter is ready
     * @return True if ready
     */
    bool isReady();

    /*
     * @brief reset Resets the filter. After reset(), the
     *    filter needs to be re-initialized.
     */
    void reset();

    /*
     * @brief prediction Performs the predictions step
     *    of the Kalman filter
     * @param curr_time_stamp Current time
     */
    void prediction(const double& curr_time_stamp);

    /*
     * @brief update Performs the measurement update step
     *    of the Kalman filter
     * @param m_attitude Measured atttiude from the motion
     *    capture system
     * @param m_position Measured posiiton from the motion
     *    capture system
     */
    void update(const Eigen::Quaterniond& m_attitude,
        const Eigen::Vector3d& m_position);

  private:
    // Disable copy constructor and assign operator for the class
    KalmanFilter(const KalmanFilter&);
    KalmanFilter& operator=(const KalmanFilter&);

    // Status of the filter during initialization
    // The filter requires two sets of measurements in order
    // to compute a good initial pose for the rigid body
    enum Status{
      INIT_POSE,
      INIT_TWIST,
      READY
    };

    // Status of the filter
    Status filter_status;

    // Time that the state correspond to
    double last_time_stamp;
    // Expected time interval between updates
    double msg_interval;

    // Jacobians
    Matrix12d   proc_jacob;       // Process jacobian
    Matrix6_12d meas_jacob;       // Measurement jacobian
    Matrix12d   proc_noise_jacob; // Process noise jacobian
    Matrix6d    meas_noise_jacob; // Measurement noise jacobian
};

}

#endif
