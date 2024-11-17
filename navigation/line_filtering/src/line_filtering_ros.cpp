#include "line_filtering/line_filtering_ros.hpp"

Camera3DPointsNode::Camera3DPointsNode() : Node("camera_3d_points_node")
{
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos_sensor_data = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    // Subscriptions
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/cam_down/camera_info", qos_sensor_data, 
        std::bind(&Camera3DPointsNode::cameraInfoCallback, this, std::placeholders::_1));
    
    depth_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/dvl/depth", qos_sensor_data, 
        std::bind(&Camera3DPointsNode::depthCallback, this, std::placeholders::_1));
        
    pose_array_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/line/pose_array", qos_sensor_data,
        std::bind(&Camera3DPointsNode::poseArrayCallback, this, std::placeholders::_1));

    point_1_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/line/point_1", qos_sensor_data);
    point_2_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/line/point_2", qos_sensor_data);
    point_3_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/line/point_3", qos_sensor_data);
    point_4_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/line/point_4", qos_sensor_data);


    
    depth_.data = -1.0;

    //publisher
    pose_array_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("pose_array", qos_sensor_data);
}

void Camera3DPointsNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    K_ = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();
    RCLCPP_INFO(this->get_logger(), "Camera Intrinsic Matrix initialized.");
    camera_info_received_ = true;
    camera_info_sub_.reset();
}



void Camera3DPointsNode::poseArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    if (depth_.data < 0.0) {
        RCLCPP_WARN(this->get_logger(), "Depth data not yet received.");
        return;
    }
    if (!camera_info_received_) {
        RCLCPP_WARN(this->get_logger(), "Camera info not yet received.");
        return;
    }

    double depth = depth_.data;

    RCLCPP_INFO(this->get_logger(), "Received PoseArray with %zu poses", msg->poses.size());

    // Create a new PoseArray to publish
    geometry_msgs::msg::PoseArray updated_pose_array;
    updated_pose_array.header = msg->header;  // Use the same header for consistency

    // Iterate through the incoming poses and compute 3D points
    size_t i = 0;
    for (const auto &pose : msg->poses) {
        geometry_msgs::msg::Pose updated_pose;

        // Extract x and y from the incoming PoseArray
        double u = pose.position.x;
        double v = pose.position.y;

        // Transform 2D (u, v) to 3D (X, Y, Z) using the camera intrinsic matrix
        double X = (u - K_.at<double>(0, 2)) * depth / K_.at<double>(0, 0);
        double Y = (v - K_.at<double>(1, 2)) * depth / K_.at<double>(1, 1);

        // Create the updated pose
        updated_pose.position.x = X;
        updated_pose.position.y = Y;
        updated_pose.position.z = depth;  // Use the depth value from DVL
        updated_pose.orientation = pose.orientation;  // Retain the orientation from the incoming pose

        // Add the updated pose to the PoseArray
        updated_pose_array.poses.push_back(updated_pose);

        // Publish individual PoseStamped messages for specific points (optional)
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header = msg->header;
        pose_msg.pose = updated_pose;

        switch (i) {
            case 0: point_1_->publish(pose_msg); break;
            case 1: point_2_->publish(pose_msg); break;
            case 2: point_3_->publish(pose_msg); break;
            case 3: point_4_->publish(pose_msg); break;
            default: break;
        }

        RCLCPP_INFO(this->get_logger(), "Point %zu: 2D (%f, %f) -> 3D (%f, %f, %f)", 
                    i + 1, u, v, X, Y, depth);

        i++;
    }

    // Publish the updated PoseArray
    pose_array_pub_->publish(updated_pose_array);

    RCLCPP_INFO(this->get_logger(), "Published updated PoseArray with %zu poses", updated_pose_array.poses.size());
}

void Camera3DPointsNode::depthCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    depth_.data = msg->data;
    RCLCPP_INFO(this->get_logger(), "Received depth: %f", msg->data);
}

// LineGrouper lines_grouped(double point1_, double point2_, 
//     double point3_ = 0, double point4_  = 0)
// {
//         LineGrouper lines_grouped_temp;

//         lines_grouped_temp.line1_.start.x= point1.x;
//         lines_grouped_temp.line1_.start.y = point1.y;

//         lines_grouped_temp.line1_.end.x = point2.x;
//         lines_grouped_temp.line1_.end.y = point2.y;

//         lines_grouped_temp.line2_.start.x = point3.x;
//         lines_grouped_temp.line2_.start.y = point3.y;

//         lines_grouped_temp.line2_.end.x = point4.x;
//         lines_grouped_temp.line2_.end.y = point4.y;

//         return lines_grouped_temp;

// }


// //Selecting line based on Y values
// LineGrouper line_selector(lines_combined.line1_ line1, lines_combined.line2 line2)
//  { 
//     if(line1.start.y > line2.end.y)
//     {
//         return line1;
//     }
//     else{
//         return line2;
//     }
//  }