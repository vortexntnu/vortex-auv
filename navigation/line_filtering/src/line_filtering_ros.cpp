#include"line_filtering.hpp"

Camera3DPointsNode::Camera3DPointsNode() : Node("camera_3d_points_node")
{
    image_sub_ = image_transport::create_subscription(
        this, "/zed/zed_node/left/image_rect_color", std::bind(&Camera3DPointsNode::imageCallback, this, std::placeholders::_1), "raw");

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/zed/zed_node/left/camera_info", 10, std::bind(&Camera3DPointsNode::cameraInfoCallback, this, std::placeholders::_1));

    point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("camera_3d_points", 10);
    upper_left_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("upper_left", 10);
    upper_right_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("upper_right", 10);
    lower_left_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("lower_left", 10);
    lower_right_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("lower_right", 10);
    pointmiddle_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("middle", 10);
}

void Camera3DPointsNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    K_ = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();
    RCLCPP_INFO(this->get_logger(), "Camera Intrinsic Matrix initialized.");
}

void Camera3DPointsNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
    if (K_.empty()) {
        RCLCPP_WARN(this->get_logger(), "Camera intrinsic matrix not initialized yet.");
        return;
    }

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

    double Z = 0.8;
    std::vector<cv::Point3f> points_3d;
    for (size_t i = 0; i < image_points.size(); ++i) {
        double X = (image_points[i].x - K_.at<double>(0, 2)) * Z / K_.at<double>(0, 0);
        double Y = (image_points[i].y - K_.at<double>(1, 2)) * Z / K_.at<double>(1, 1);

        points_3d.push_back(cv::Point3f(X, Y, Z));
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header = msg->header;
        pose_msg.pose.position.x = X;
        pose_msg.pose.position.y = Y;
        pose_msg.pose.position.z = Z;
        pose_msg.pose.orientation.w = 1.0;

        switch (i) {
            case 0: upper_left_->publish(pose_msg); break;
            case 1: upper_right_->publish(pose_msg); break;
            case 2: lower_left_->publish(pose_msg); break;
            case 3: lower_right_->publish(pose_msg); break;
            case 4: pointmiddle_->publish(pose_msg); break;
        }

        RCLCPP_INFO(this->get_logger(), "Point %zu: 2D (%f, %f) -> 3D (%f, %f, %f)", 
                    i + 1, image_points[i].x, image_points[i].y, X, Y, Z);
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