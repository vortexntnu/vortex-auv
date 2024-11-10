#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class Camera3DPointsNode : public rclcpp::Node
{
public:
    Camera3DPointsNode() : Node("camera_3d_points_node")
    {
        // Subscribe to the image and camera info topics
        image_sub_ = image_transport::create_subscription(
            this, "/zed/zed_node/left/image_rect_color", std::bind(&Camera3DPointsNode::imageCallback, this, std::placeholders::_1), "raw");

        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/zed/zed_node/left/camera_info", 10, std::bind(&Camera3DPointsNode::cameraInfoCallback, this, std::placeholders::_1));
        
        // Initialize the PointCloud2 publisher for visualization
        point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("camera_3d_points", 10);

        // Initialize publishers for each unique 3D point
        upper_left_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("upper_left", 10);
        upper_right_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("upper_right", 10);
        lower_left_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("lower_left", 10);
        lower_right_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("lower_right", 10);
        pointmiddle_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("middle", 10);
    }

private:
    image_transport::Subscriber image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
    
    // Publishers for each unique 3D point as PoseStamped
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr upper_left_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr upper_right_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr lower_left_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr lower_right_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pointmiddle_;
    
    cv::Mat K_; // Camera intrinsic matrix

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        // Extract the intrinsic camera matrix (K) from the CameraInfo message
        K_ = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();
        RCLCPP_INFO(this->get_logger(), "Camera Intrinsic Matrix initialized.");
    }

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
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

    // Define the 2D points to be transformed to 3D points
    std::vector<cv::Point2f> image_points = {
        cv::Point2f(0, 0),              // Top-left
        cv::Point2f(cv_ptr->image.cols - 1, 0),    // Top-right
        cv::Point2f(0, cv_ptr->image.rows - 1),    // Bottom-left
        cv::Point2f(cv_ptr->image.cols - 1, cv_ptr->image.rows - 1), // Bottom-right
        cv::Point2f(cv_ptr->image.cols / 2.0, cv_ptr->image.rows / 2.0) // Center
    };

    double Z = 0.8; // Fixed depth value for simplicity

    // Calculate 3D points and publish as PoseStamped for each of the five points
    std::vector<cv::Point3f> points_3d;  // Define points_3d to hold the 3D points
    for (size_t i = 0; i < image_points.size(); ++i) {
        double X = (image_points[i].x - K_.at<double>(0, 2)) * Z / K_.at<double>(0, 0);
        double Y = (image_points[i].y - K_.at<double>(1, 2)) * Z / K_.at<double>(1, 1);

        // Store the calculated 3D point in points_3d
        points_3d.push_back(cv::Point3f(X, Y, Z));

        // Prepare PoseStamped message
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header = msg->header; // Use the image header for timestamp and frame_id
        pose_msg.pose.position.x = X;
        pose_msg.pose.position.y = Y;
        pose_msg.pose.position.z = Z;
        pose_msg.pose.orientation.w = 1.0; // Identity quaternion since orientation is not calculated

        // Publish the PoseStamped message to the corresponding topic
        switch (i) {
            case 0:
                upper_left_->publish(pose_msg);
                break;
            case 1:
                upper_right_->publish(pose_msg);
                break;
            case 2:
                lower_left_->publish(pose_msg);
                break;
            case 3:
                lower_right_->publish(pose_msg);
                break;
            case 4:
                pointmiddle_->publish(pose_msg);
                break;
        }

        // Log the 3D point
        RCLCPP_INFO(this->get_logger(), "Point %zu: 2D (%f, %f) -> 3D (%f, %f, %f)", 
                    i + 1, image_points[i].x, image_points[i].y, X, Y, Z);
    }

    // Optionally, create and publish a PointCloud2 message for visualization
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
    point_cloud_msg.fields[0].count = 1;

    point_cloud_msg.fields[1].name = "y";
    point_cloud_msg.fields[1].offset = 4;
    point_cloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    point_cloud_msg.fields[1].count = 1;

    point_cloud_msg.fields[2].name = "z";
    point_cloud_msg.fields[2].offset = 8;
    point_cloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    point_cloud_msg.fields[2].count = 1;

    point_cloud_msg.point_step = 12;
    point_cloud_msg.row_step = point_cloud_msg.point_step * point_cloud_msg.width;
    point_cloud_msg.data.resize(point_cloud_msg.row_step);

    sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud_msg, "z");

    for (const auto &pt : points_3d) {
        *iter_x = pt.x;
        *iter_y = pt.y;
        *iter_z = pt.z;
        ++iter_x; ++iter_y; ++iter_z;
    }

    point_cloud_pub_->publish(point_cloud_msg);
}
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Camera3DPointsNode>());
    rclcpp::shutdown();
    return 0;
}
