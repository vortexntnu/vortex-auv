#ifndef EKF_POSE_FILTERING_ROS_HPP
#define EKF_POSE_FILTERING_ROS_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/buffer.h>
#include <message_filters/subscriber.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/create_timer_ros.h>
#include <std_srvs/srv/set_bool.hpp>
#include <Eigen/Dense>
#include <vortex_filtering/vortex_filtering.hpp>
#include <tuple>

using Vector6d = Eigen::Vector<double, 3>;

class EKFPoseFilteringNode : public rclcpp::Node{

    public:

        EKFPoseFilteringNode();

        ~EKFPoseFilteringNode(){};


    private:
        void resetEFK(std::shared_ptr<std_srvs::srv::SetBool::Response> response);

        void pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_msg);

        void filter_pose(geometry_msgs::msg::PoseStamped &transformed_pose);

        geometry_msgs::msg::Quaternion enu_to_ned_quaternion(const geometry_msgs::msg::Quaternion& enu_quat);

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

        using DynMod = vortex::models::ConstantPose;
        using SensMod = vortex::models::IdentitySensorModel<4,4>;
        using EKF = vortex::filter::EKF<DynMod, SensMod>;
        using Gauss4d = vortex::prob::Gauss<4>;

        bool first_run_ = true;
        rclcpp::Time previous_time_;
    
        std::shared_ptr<DynMod> dynamic_model_;
        std::shared_ptr<SensMod> sensor_model_;
        
        Gauss4d previous_pose_est_; 
        Gauss4d object_pose_est_;

        std::string frame_;
        bool enu_orientation_;

};
    
#endif //EKF_POSE_FILTERING_ROS_HPP