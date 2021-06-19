/*
 * Copyright [2015]
 * [Kartik Mohta <kartikmohta@gmail.com>]
 * [Ke Sun <sunke.polyu@gmail.com>]
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

#ifndef QUALISYS_DRIVER_H
#define QUALISYS_DRIVER_H

#include <cmath>
#include <string>
#include <set>
#include <vector>
#include <boost/asio.hpp>
#include <ros/ros.h>
#include <mocap_base/MoCapDriverBase.h>
#include <mocap_qualisys/RTProtocol.h>



namespace mocap{

class QualisysDriver: public MoCapDriverBase{

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /*
     * @brief Constructor
     * @param nh Ros node
     */
    QualisysDriver(const ros::NodeHandle& n):
      MoCapDriverBase   (n),
      max_accel         (10.0),
      frame_interval    (0.01),
      last_packet_time  (0),
      process_noise     (Eigen::Matrix<double, 12, 12>::Zero()),
      measurement_noise (Eigen::Matrix<double, 6, 6>::Zero())
      {
      return;
    }

    /*
     * @brief Destructor
     */
    ~QualisysDriver() {
      disconnect();
      return;
    }

    /*
     * @brief init Initialize the object
     * @return True if successfully initialized
     */
    bool init();

    /*
     * @brief run Start acquiring data from the server
     */
    bool run();

    /*
     * @brief disconnect Disconnect to the server
     * @Note The function is called automatically when the
     *  destructor is called.
     */
    void disconnect();

  private:
    // Disable the copy constructor and assign operator
    QualisysDriver(const QualisysDriver& );
    QualisysDriver& operator=(const QualisysDriver& );

    // Handle a frame which contains the info of all subjects
    void handleFrame();

    // Handle a the info of a single subject
    void handleSubject(int sub_idx);
    // void handleSubject(int sub_idx);

    // Unit converter
    static double deg2rad;

    // Port of the server to be connected
    int base_port;

    // Port that the server should stream UDP packets to.
    // 0 indicates that TCP is used.
    unsigned short udp_stream_port;

    // Minor version of the QTM protocol
    int qtm_protocol_version;

    // Protocol to connect to the server
    CRTProtocol port_protocol;

    // A pointer to the received packet
    // (no need to initialize)
    CRTPacket* prt_packet;

    // A set to hold the model names
    std::set<std::string> model_set;

    // Max acceleration
    double max_accel;

    // Average time interval between two frames
    double frame_interval;

    // time point for last frame
    unsigned long last_packet_time;

    // Convariance matrices for initializing kalman filters
    Eigen::Matrix<double, 12, 12> process_noise;
    Eigen::Matrix<double,  6,  6> measurement_noise;

    // Timestamp stuff
    double start_time_local_ = 0;
    double start_time_packet_ = 0;
};
}


#endif