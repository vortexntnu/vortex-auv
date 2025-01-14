#ifndef LINE_FILTERING_ROS_HPP
#define LINE_FILTERING_ROS_HPP

#include <chrono>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>  
#include <std_msgs/msg/float64.hpp> 
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/pose.hpp>


          
       

class Camera3DPointsNode : public rclcpp::Node
{
public:
    Camera3DPointsNode();

    struct PointGrouper{
        geometry_msgs::msg::Point start;
        geometry_msgs::msg::Point end;
    };

    //struct to store both lines together
    struct LineGrouper{
        PointGrouper line1_;
        PointGrouper line2_;
    };

    LineGrouper lines_combined;

    //selects which line is the current one
    LineGrouper line_selector(PointGrouper& line1, PointGrouper& line2);


    //function for grouping two points into a line
    //might have to run recursive
    LineGrouper lines_grouped(geometry_msgs::msg::Point point1_, geometry_msgs::msg::Point point2_, geometry_msgs::msg::Point point3_, geometry_msgs::msg::Point point4_);


private:
    //Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr depth_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_sub_;

    //Publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr point_1_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr point_2_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr point_3_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr point_4_;

    geometry_msgs::msg::PoseArray::SharedPtr linePointsArray_;
    geometry_msgs::msg::PoseArray::SharedPtr odomLinePointsArray_;

    bool camera_info_received_ = false;
    cv::Mat K_; 

    std_msgs::msg::Float64 depth_;

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
    void depthCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void poseArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg); 

     //timer

    void timer_callback();
    void update_timer(int update_interval);
    rclcpp::TimerBase::SharedPtr timer_;



};

#endif  // CAMERA_3D_POINTS_NODE_HPP