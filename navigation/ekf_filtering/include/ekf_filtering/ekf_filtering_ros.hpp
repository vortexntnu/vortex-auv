#ifndef EKF_FILTER_ROS_HPP
#define EKF_FILTER_ROS_HPP

#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "message_filters/subscriber.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"
#include "src/vortex-aruco-detection/aruco-detector/include/aruco_detector/aruco_detector_ros.hpp"


#endif
/* Requirements
- A node that subscribes to a node for the kalman_callback which gives the local coordinates as 6DOF
- A node that publishes coordinates in global frame
- We only want the node to run when the object in local frame is actually visible in the camera (callback)
- function (local_frame_pos, auv_global_pos)
- Save previous value to use for ekf step

- For ekf:step we need: (set global names)
         time_since_previous callback, 
         board_pose_est - just call it, we get it from gauss. -> object
         board_pose_meas (measuement) - from BoardPoaseStamp -> object
         from topic we listen) 

    For dynmod -  vortex::models::IdentityDynamicModel<6>;


- using DynMod, and 

From aruco_detector_ros_hpp
    using DynMod = vortex::models::IdentityDynamicModel<6>;
    using SensMod = vortex::models::IdentitySensorModel<6,6>;
    using EKF = vortex::filter::EKF<DynMod, SensMod>;

    
    std::shared_ptr<DynMod> dynamic_model_;
    std::shared_ptr<SensMod> sensor_model_;

*/


using std::chrono_literals;

namespace vortex
{
namespace ekf_filtering

{
class EkfFilteringNode : public rclcpp::Node {
    private:
        std::string target_frame_; //Frame to map the local frame data
        std::shared_ptr<tf2_ros::Buffer> tf2_buffer_; //
        std::shared_ptr<tf2_ros::TransformListener> tf2_listener_; //
        message_filters::Subscriber<geometry_msgs::msg::PointStamped> point_sub_; //
        std::shared_ptr<tf2_ros::MessageFilter<geometry_msgs::msg::PointStamped>> tf2_filter_; //

        using DynMod = vortex::models::IdentityDynamicModel<6>;
        using SensMod = vortex::models::IdentitySensorModel<6,6>;
        using EKF = vortex::filter::EKF<DynMod, SensMod>;


    public:
        void ekf_filtering::EkfFilteringNode::EkfFilterCallback()
        {







        }








}







} //std::chrono_literals
} //ekf_filtering