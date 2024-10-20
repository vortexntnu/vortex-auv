#ifndef EKF_FILTERING_ROS_HPP
#define EKF_FILTERING_ROS_HPP

#include <chrono>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "message_filters/subscriber.h"
#include "rclcpp/parameter_event_handler.hpp"
#include <rclcpp/qos.hpp>
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"
#include <tuple>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vortex_filtering/vortex_filtering.hpp>



#include <vector>
//#include <opencv2/opencv.hpp>



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

For KallmanFilterCallback function:
    getBoardPoseStamp(); - Need this function

*/


using std::placeholders::_1;


namespace vortex
{

namespace ekf_filtering

{

class EkfFilteringNode : public rclcpp::Node{

    public:

        EkfFilteringNode();

        ~EkfFilteringNode(){};

    private:
    
        std::string target_frame_; //Frame to map the local frame data
        std::shared_ptr<tf2_ros::Buffer> tf2_buffer_; //
        std::shared_ptr<tf2_ros::TransformListener> tf2_listener_; //
        message_filters::Subscriber<geometry_msgs::msg::PointStamped> point_sub_; //
        std::shared_ptr<tf2_ros::MessageFilter<geometry_msgs::msg::PointStamped>> tf2_filter_; //

        

        using DynMod = vortex::models::IdentityDynamicModel<6>;
        using SensMod = vortex::models::IdentitySensorModel<6,6>;
        using EKF = vortex::filter::EKF<DynMod, SensMod>;

        static rclcpp::Time previous_time;
        rclcpp::Time current_time;
        rclcpp::Duration time_since_previous_callback;
        bool callback_flag;
        

        void kalmanFilterCallback();
        void poseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_msg);


        std::shared_ptr<DynMod> dynamic_model_;
        std::shared_ptr<SensMod> sensor_model_;
        vortex::prob::Gauss<6> board_pose_est_;


        std::string target_frame_;
        std::shared_ptr<tf2_ros::Buffer> tf2_buffer;
        std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
        std::chrono::duration<int> buffer_timeout;

        // Subscriber and message filter for the input PoseStamped messages
        message_filters::Subscriber<geometry_msgs::msg::PoseStamped> pose_sub_;
        std::shared_ptr<tf2_ros::MessageFilter<geometry_msgs::msg::PoseStamped>> tf2_filter_;

        // Publisher for the transformed poses
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr transformed_pose_pub



} 
} //namespace ekf_filtering
} //namespace vortex

#endif