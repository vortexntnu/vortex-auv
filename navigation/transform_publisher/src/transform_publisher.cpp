#include "transform_publisher/transform_publisher.hpp"

TransformPublisher::TransformPublisher()
    : Node("transform_publisher"), transform_received_(false) {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos_sensor_data = rclcpp::QoS(
        rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    pose_sub_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/orca/pose", qos_sensor_data,
        std::bind(&TransformPublisher::poseCallback, this,
                  std::placeholders::_1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "odom_base_link", qos_sensor_data);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Wait and get the static transform between dvl_link and base_link
    while (rclcpp::ok() && !transform_received_) {
        try {
            static_transform_ = tf_buffer_->lookupTransform(
                "dvl_link", "base_link", tf2::TimePointZero,
                tf2::durationFromSec(1.0));
            transform_received_ = true;
            RCLCPP_INFO(this->get_logger(), "Static transform received.");
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "Waiting for static transform: %s",
                        ex.what());
            rclcpp::sleep_for(std::chrono::seconds(1));
        }
    }
}

void TransformPublisher::poseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    if (!transform_received_) {
        RCLCPP_WARN(this->get_logger(), "Static transform not yet received.");
        return;
    }

    nav_msgs::msg::Odometry transformed_odom;
    transformed_odom.header.stamp = msg->header.stamp;
    transformed_odom.header.frame_id = "base_link";

    geometry_msgs::msg::PoseStamped pose_in, pose_out;
    pose_in.header = msg->header;
    pose_in.pose = msg->pose.pose;

    try {
        tf2::doTransform(pose_in, pose_out, static_transform_);
        transformed_odom.pose.pose = pose_out.pose;
        transformed_odom.pose.covariance = msg->pose.covariance;

        odom_pub_->publish(transformed_odom);

        geometry_msgs::msg::TransformStamped dynamic_transform;
        dynamic_transform.header.stamp = this->get_clock()->now();
        dynamic_transform.header.frame_id = "odom";
        dynamic_transform.child_frame_id = "base_link";

        dynamic_transform.transform.translation.x =
            transformed_odom.pose.pose.position.x;
        dynamic_transform.transform.translation.y =
            transformed_odom.pose.pose.position.y;
        dynamic_transform.transform.translation.z =
            transformed_odom.pose.pose.position.z;
        dynamic_transform.transform.rotation =
            transformed_odom.pose.pose.orientation;

        tf_broadcaster_->sendTransform(dynamic_transform);
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform pose: %s",
                    ex.what());
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TransformPublisher>());
    rclcpp::shutdown();
    return 0;
}
