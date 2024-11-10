#ifndef CAMERA_3D_POINTS_NODE_HPP
#define CAMERA_3D_POINTS_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>                
#include <stonefish_ros2/msg/dvl_beam.hpp>      // For stonefish_ros2::msg::DVLBeam (adjust if needed)
       

class Camera3DPointsNode : public rclcpp::Node
{
public:
    Camera3DPointsNode();

private:
    image_transport::Subscriber image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr upper_left_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr upper_right_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr lower_left_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr lower_right_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pointmiddle_;

    cv::Mat K_; // Camera intrinsic matrix

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
};

#endif  // CAMERA_3D_POINTS_NODE_HPP