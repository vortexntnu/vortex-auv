#include "odom_transformer/odom_transformer.hpp"
#include <rclcpp_components/register_node_macro.hpp>

OdomTransformer::OdomTransformer(const rclcpp::NodeOptions& options)
    : Node("odom_transformer_node", options) {
    frame_prefix_ = this->declare_parameter<std::string>("frame_prefix");
    if (!frame_prefix_.empty() && frame_prefix_.back() == '/') {
        frame_prefix_.pop_back();
    }

    sensor_frame_ = this->declare_parameter<std::string>("sensor_frame");
    publish_tf_ = this->declare_parameter<bool>("publish_tf");
    publish_pose_ = this->declare_parameter<bool>("publish_pose");
    publish_twist_ = this->declare_parameter<bool>("publish_twist");

    if (publish_tf_) {
        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    this->declare_parameter<std::string>("topics.input");
    this->declare_parameter<std::string>("topics.output");
    this->declare_parameter<std::string>("topics.pose");
    this->declare_parameter<std::string>("topics.twist");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&OdomTransformer::lookup_static_transforms, this));
}

void OdomTransformer::lookup_static_transforms() {
    try {
        auto tf = tf2::transformToEigen(tf_buffer_->lookupTransform(
            frame("base_link"), frame(sensor_frame_), tf2::TimePointZero));
        R_base_sensor_ = tf.rotation();
        t_base_sensor_ = tf.translation();

        tf_loaded_ = true;
        tf_timer_->cancel();

        Eigen::Vector3d rpy =
            R_base_sensor_.eulerAngles(2, 1, 0).reverse() * 180.0 / M_PI;
        RCLCPP_INFO(get_logger(),
                    "Loaded static transform: %s -> %s  "
                    "t=(%.3f, %.3f, %.3f)  rpy=(%.1f, %.1f, %.1f) deg",
                    frame("base_link").c_str(), frame(sensor_frame_).c_str(),
                    t_base_sensor_.x(), t_base_sensor_.y(), t_base_sensor_.z(),
                    rpy.x(), rpy.y(), rpy.z());
        complete_initialization();
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(get_logger(), "TF lookup failed (will retry): %s",
                    ex.what());
    }
}

void OdomTransformer::complete_initialization() {
    auto qos = rclcpp::QoS(1).best_effort();

    auto input_topic = this->get_parameter("topics.input").as_string();
    auto output_topic = this->get_parameter("topics.output").as_string();

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        input_topic, qos,
        std::bind(&OdomTransformer::odom_callback, this,
                  std::placeholders::_1));

    odom_pub_ =
        this->create_publisher<nav_msgs::msg::Odometry>(output_topic, qos);

    if (publish_pose_) {
        pose_pub_ = this->create_publisher<
            geometry_msgs::msg::PoseWithCovarianceStamped>(
            this->get_parameter("topics.pose").as_string(), qos);
    }

    if (publish_twist_) {
        twist_pub_ = this->create_publisher<
            geometry_msgs::msg::TwistWithCovarianceStamped>(
            this->get_parameter("topics.twist").as_string(), qos);
    }

    reset_origin_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "reset_odom_origin",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
            origin_set_ = false;
            response->success = true;
            response->message =
                "Odom origin reset; next message sets new origin.";
            RCLCPP_INFO(get_logger(), "Odom origin reset requested.");
        });

    RCLCPP_INFO(get_logger(), "Odom transformer: %s -> %s", input_topic.c_str(),
                output_topic.c_str());
}

void OdomTransformer::odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Current orientation of the sensor in odom frame
    Eigen::Quaterniond q_odom_sensor(
        msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

    // Velocities in sensor frame
    Eigen::Vector3d v_sensor(msg->twist.twist.linear.x,
                             msg->twist.twist.linear.y,
                             msg->twist.twist.linear.z);
    Eigen::Vector3d omega_sensor(msg->twist.twist.angular.x,
                                 msg->twist.twist.angular.y,
                                 msg->twist.twist.angular.z);

    Eigen::Matrix3d R_odom_sensor = q_odom_sensor.toRotationMatrix();

    // Orientation: R_odom_base = R_odom_sensor * R_base_sensor^-1
    Eigen::Matrix3d R_odom_base = R_odom_sensor * R_base_sensor_.transpose();

    // Position: p_base = p_sensor - R_odom_base * t_base_sensor
    Eigen::Vector3d p_sensor(msg->pose.pose.position.x,
                             msg->pose.pose.position.y,
                             msg->pose.pose.position.z);
    Eigen::Vector3d p_base = p_sensor - R_odom_base * t_base_sensor_;

    // Capture the first base_link pose as the odom frame origin
    if (!origin_set_) {
        double yaw0 = std::atan2(R_odom_base(1, 0), R_odom_base(0, 0));

        R_origin_ = Eigen::AngleAxisd(yaw0, Eigen::Vector3d::UnitZ())
                        .toRotationMatrix();

        p_origin_ = p_base;
        origin_set_ = true;
    }

    // Express pose relative to initial position and yaw, preserving odometry
    // roll/pitch
    Eigen::Matrix3d R_out = R_origin_.transpose() * R_odom_base;
    Eigen::Vector3d p_out = R_origin_.transpose() * (p_base - p_origin_);
    Eigen::Quaterniond q_out(R_out);
    q_out.normalize();

    // Angular velocity: rotate from sensor frame to base_link frame
    Eigen::Vector3d omega_base = R_base_sensor_ * omega_sensor;

    // Linear velocity: lever arm correction
    // v_base = R_base_sensor * v_sensor - omega_base x t_base_sensor
    Eigen::Vector3d v_base =
        R_base_sensor_ * v_sensor - omega_base.cross(t_base_sensor_);

    // Build output odometry
    auto out = std::make_unique<nav_msgs::msg::Odometry>();
    out->header.stamp = msg->header.stamp;
    out->header.frame_id = frame("odom");
    out->child_frame_id = frame("base_link");

    out->pose.pose.position.x = p_out.x();
    out->pose.pose.position.y = p_out.y();
    out->pose.pose.position.z = p_out.z();
    out->pose.pose.orientation.w = q_out.w();
    out->pose.pose.orientation.x = q_out.x();
    out->pose.pose.orientation.y = q_out.y();
    out->pose.pose.orientation.z = q_out.z();

    out->twist.twist.linear.x = v_base.x();
    out->twist.twist.linear.y = v_base.y();
    out->twist.twist.linear.z = v_base.z();
    out->twist.twist.angular.x = omega_base.x();
    out->twist.twist.angular.y = omega_base.y();
    out->twist.twist.angular.z = omega_base.z();

    out->pose.covariance = msg->pose.covariance;
    out->twist.covariance = msg->twist.covariance;

    if (pose_pub_) {
        auto pose_msg =
            std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>();
        pose_msg->header = out->header;
        pose_msg->pose = out->pose;
        pose_pub_->publish(std::move(pose_msg));
    }

    if (twist_pub_) {
        auto twist_msg =
            std::make_unique<geometry_msgs::msg::TwistWithCovarianceStamped>();
        twist_msg->header = out->header;
        twist_msg->header.frame_id = out->child_frame_id;
        twist_msg->twist = out->twist;
        twist_pub_->publish(std::move(twist_msg));
    }

    if (tf_broadcaster_) {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = msg->header.stamp;
        tf_msg.header.frame_id = frame("odom");
        tf_msg.child_frame_id = frame("base_link");
        tf_msg.transform.translation.x = p_out.x();
        tf_msg.transform.translation.y = p_out.y();
        tf_msg.transform.translation.z = p_out.z();
        tf_msg.transform.rotation = out->pose.pose.orientation;
        tf_broadcaster_->sendTransform(tf_msg);
    }

    odom_pub_->publish(std::move(out));
}

RCLCPP_COMPONENTS_REGISTER_NODE(OdomTransformer)
