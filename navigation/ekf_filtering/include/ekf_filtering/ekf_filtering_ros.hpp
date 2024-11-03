#ifndef EKF_FILTERING_ROS_HPP
#define EKF_FILTERING_ROS_HPP

#include <chrono>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

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
#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vortex_filtering/vortex_filtering.hpp>



//#include <opencv2/opencv.hpp>



/* Requirements

    

*/

namespace vortex::ekf_filtering

{
using Vector6d = Eigen::Vector<double, 6>;

class EKFFilteringNode : public rclcpp::Node{

    public:

        EKFFilteringNode();

        ~EKFFilteringNode(){};


    private:
        //first run callback
        void SetFirstRunCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, 
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);

        //Creating the service
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
    
        std::string target_frame_;
        std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

        // Subscriber and message filter for the input PoseStamped messages
        message_filters::Subscriber<geometry_msgs::msg::PoseStamped> pose_sub_;
        std::shared_ptr<tf2_ros::MessageFilter<geometry_msgs::msg::PoseStamped>> tf2_filter_;


        // Publisher for the transformed poses
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr transformed_pose_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr filtered_pose_pub_;

    /**
     * @brief Struct to store the board pose, status and timestamp using in kalmanFilterCallback function.
    */ 
        // struct BoardPoseStamp{
        // std::tuple<BoardDetectionStatus, Vector6d, rclcpp::Time> getBoardPoseStamp() const {
        // return {board_detection_status_, board_pose_meas_, stamp_};
        // }
        struct ObjectPoseStamp{

        ObjectPoseStamp() : object_pose_meas_(Eigen::Matrix<double,6,1>::Zero()), stamp_(0,0) {}
        
        ObjectPoseStamp(const Eigen::Matrix<double,6,1>& pose, const rclcpp::Time& time) : object_pose_meas_(pose),stamp_(time) {}
        
        std::tuple<Vector6d, rclcpp::Time> getObjectPoseStamp() const {
        return {object_pose_meas_, stamp_};
         }
        private:
            Eigen::Matrix<double,6,1> object_pose_meas_ = Eigen::Matrix<double,6,1>::Zero();
            rclcpp::Time stamp_;
        };
        

        using DynMod = vortex::models::IdentityDynamicModel<6>; //one for constant position as well
        using SensMod = vortex::models::IdentitySensorModel<6,6>;
        using EKF = vortex::filter::EKF<DynMod, SensMod>;


        bool first_run_ = true;
        rclcpp::Time previous_time_;
    
        rclcpp::Duration time_since_previous_callback;
        //bool poseCallback;
        void poseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_msg);
        geometry_msgs::msg::PoseStamped kalmanFilterCallback(geometry_msgs::msg::PoseStamped transformed_pose);
        


        std::shared_ptr<DynMod> dynamic_model_;
        std::shared_ptr<SensMod> sensor_model_;
        
        vortex::prob::Gauss<6> previous_pose_est_; 
        vortex::prob::Gauss<6> object_pose_est_;
        geometry_msgs::msg::PoseStamped estimated_pose_;
        Vector6d transformed_pose_vector_ = Vector6d::Zero(6);

        rclcpp::Time previous_est_time_;
        rclcpp::Time stamp_;

        std::vector<int64_t> ids_;
        std::unordered_map<int,int> id_detection_counter_;
        std::string frame_;
        ObjectPoseStamp object_measurement_;
        vortex::prob::Gauss<6> board_pose_est_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr timeout_timer_;
        std::string image_topic_;
        std::string camera_info_topic_;



        double ekf_x_pos_;
        double ekf_y_pos_;
        double ekf_z_pos_;

        double ekf_x_orientation_;
        double ekf_y_orientation_; //
        double ekf_z_orientation_; //
        


};


} //namespace vortex::ekf_filtering
#endif