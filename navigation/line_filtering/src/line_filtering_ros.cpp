#include "line_filtering/line_filtering_ros.hpp"

Camera3DPointsNode::Camera3DPointsNode() : Node("camera_3d_points_node")
{
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos_sensor_data = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    image_sub_ = image_transport::create_subscription(
        this, "/cam_down/image_color", std::bind(&Camera3DPointsNode::imageCallback, this, std::placeholders::_1), "raw");

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/cam_down/camera_info", qos_sensor_data, std::bind(&Camera3DPointsNode::cameraInfoCallback, this, std::placeholders::_1));
    
   depth_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/dvl/depth", qos_sensor_data, std::bind(&Camera3DPointsNode::depthCallback, this, std::placeholders::_1));

    depth_.data = -1.0;

    point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("camera_3d_points", qos_sensor_data);
    upper_left_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("upper_left", qos_sensor_data);
    upper_right_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("upper_right", qos_sensor_data);
    lower_left_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("lower_left", qos_sensor_data);
    lower_right_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("lower_right", qos_sensor_data);
    pointmiddle_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("middle", qos_sensor_data);
}

void Camera3DPointsNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    K_ = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();
    RCLCPP_INFO(this->get_logger(), "Camera Intrinsic Matrix initialized.");
    camera_info_sub_.reset();
}

void Camera3DPointsNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
    if (K_.empty()) {
        RCLCPP_WARN(this->get_logger(), "Camera intrinsic matrix not initialized yet.");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Received image with size %dx%d", msg->width, msg->height);

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    std::vector<cv::Point2f> image_points = {
        cv::Point2f(0, 0),
        cv::Point2f(cv_ptr->image.cols - 1, 0),
        cv::Point2f(0, cv_ptr->image.rows - 1),
        cv::Point2f(cv_ptr->image.cols - 1, cv_ptr->image.rows - 1),
        cv::Point2f(cv_ptr->image.cols / 2.0, cv_ptr->image.rows / 2.0)
    };

    

    if (depth_.data == -1.0){
        return;
    }
    double depth = depth_.data;

    std::vector<cv::Point3f> points_3d;
    for (size_t i = 0; i < image_points.size(); ++i) {
        double X = (image_points[i].x - K_.at<double>(0, 2)) * depth / K_.at<double>(0, 0);
        double Y = (image_points[i].y - K_.at<double>(1, 2)) * depth / K_.at<double>(1, 1);

        points_3d.push_back(cv::Point3f(X, Y, depth));
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header = msg->header;
        pose_msg.pose.position.x = X;
        pose_msg.pose.position.y = Y;
        pose_msg.pose.position.z = depth;
        pose_msg.pose.orientation.w = 1.0;

    
        switch (i) {
            case 0: upper_left_->publish(pose_msg); break;
            case 1: upper_right_->publish(pose_msg); break;
            case 2: lower_left_->publish(pose_msg); break;
            case 3: lower_right_->publish(pose_msg); break;
            case 4: pointmiddle_->publish(pose_msg); break;
        }
    

        RCLCPP_INFO(this->get_logger(), "Point %zu: 2D (%f, %f) -> 3D (%f, %f, %f)", 
                    i + 1, image_points[i].x, image_points[i].y, X, Y, depth);
    }

    sensor_msgs::msg::PointCloud2 point_cloud_msg;
    point_cloud_msg.header = msg->header;
    point_cloud_msg.height = 1;
    point_cloud_msg.width = points_3d.size();
    point_cloud_msg.is_dense = true;
    point_cloud_msg.is_bigendian = false;
    point_cloud_msg.fields.resize(3);

    point_cloud_msg.fields[0].name = "x";
    point_cloud_msg.fields[0].offset = 0;
    point_cloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    }


    void Camera3DPointsNode::depthCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    depth_.data = msg->data;
    RCLCPP_INFO(this->get_logger(), "Received depth: %f", msg->data);
}