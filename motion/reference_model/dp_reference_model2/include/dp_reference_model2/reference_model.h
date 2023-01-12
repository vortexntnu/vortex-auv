#ifndef DP_REFERENCE_MODEL2_H
#define DP_REFERENCE_MODEL2_H

#include "eigen_typedefs.h"

#include <Eigen/Dense>
#include <math.h>

#include "eigen_conversions/eigen_msg.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "ros/ros.h"
// #include "tf/tf.h"
// #include "tf_conversions/tf_eigen.h"


using namespace Eigen;

class ReferenceModel{
    private:
        
        /**
        * @brief 
        */
        double zeta_1, zeta_2, zeta_3, zeta_4, zeta_5, zeta_6, zeta_7;
        double omega_1, omega_2, omega_3, omega_4, omega_5, omega_6, omega_7;

        Eigen::Matrix7d Delta;
        Eigen::Matrix7d Omega;
        Eigen::MatrixXd A_d;
        Eigen::MatrixXd B_d;
        
        /**
        * @brief Desired pose in quaternions.
        */
        Eigen::Vector7d eta_d;
        Eigen::Vector7d eta_dot_d;

        //Eigen::Vector14d 
        void calculate_smooth(Eigen::Vector7d x_ref);

        /**
        * @brief Desires the rate of the reference model.
        */
        double time_step = 0.1;


    public:
        /**
         * @brief Constructor
         *
         * @param nh ROS nodehandle
         */
        ReferenceModel(ros::NodeHandle nh);

        ros::Subscriber setpoint_sub; /* Subscriber for listening to (the guidance node ....)      */
        ros::Publisher reference_pub; /* Publisher for the DP-controller */

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