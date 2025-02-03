#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>  
#include <std_msgs/msg/float64.hpp> 
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/buffer.h>
#include <message_filters/subscriber.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/create_timer_ros.h>
#include <vortex_filtering/vortex_filtering.hpp>
#include <vortex_filtering/filters/pdaf.hpp>
#include <line_filtering/track_manager.hpp>

#include <visualization_msgs/msg/marker_array.hpp>


          
       

class LineFilteringNode : public rclcpp::Node
{
public:
    LineFilteringNode();

private:
    /**
     * @brief Updates the dynamic model with the given velocity standard deviation.
     * 
     * @param std_velocity The velocity standard deviation.
     */
    void update_dyn_model(double std_velocity);

    /**
     * @brief Updates the sensor model with the given sensor standard deviation.
     * 
     * @param std_sensor The sensor standard deviation.
     */
    void update_sensor_model(double std_sensor);

    /**
     * @brief Updates the update interval for the target tracking.
     * 
     * @param update_interval The new update interval in milliseconds.
     */
    void update_timer(int update_interval);

    /**
     * @brief Timer callback function.
     */
    void timer_callback();

    void visualize_tracks();

    Eigen::Vector2d get_line_params(Eigen::Matrix<double, 2, 2> line_points);

    //Subscriptions
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr depth_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    //Publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr point_1_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr point_2_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr point_3_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr point_4_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr line_params_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr line_points_pub_;

    std::string target_frame_;
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

    // Subscriber and message filter for the input PoseStamped messages
    message_filters::Subscriber<geometry_msgs::msg::PoseArray> line_sub_;
    std::shared_ptr<tf2_ros::MessageFilter<geometry_msgs::msg::PoseArray>> tf2_filter_;

    rclcpp::TimerBase::SharedPtr timer_;


    // only need the odom variable. Transform lines in line_callback and store them in the odom variable.
    geometry_msgs::msg::PoseArray::SharedPtr odomLinePointsArray_;

    bool camera_info_received_ = false;
    cv::Mat K_; 

    std_msgs::msg::Float64 depth_;
    geometry_msgs::msg::Pose orca_pose_;

    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void depth_callback(const std_msgs::msg::Float64::SharedPtr msg);
    void line_callback(const std::shared_ptr<const geometry_msgs::msg::PoseArray>& msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void select_line();
 

    
    // Track manager
    TrackManager track_manager_;

    Eigen::Array<double, 2, Eigen::Dynamic> measurements_;
    Eigen::Array<double, 2, Eigen::Dynamic> line_params_;

    int current_id_;
    int id_counter_;
    geometry_msgs::msg::PoseArray current_track_points_;


    bool debug_visualization_ = true;

};
