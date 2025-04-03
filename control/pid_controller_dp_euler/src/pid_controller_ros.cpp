#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <pid_controller_dp_euler/pid_controller_ros.hpp>
#include <pid_controller_dp_euler/pid_controller_utils.hpp>
#include <string>

PIDControllerNode::PIDControllerNode() : Node("pid_controller_euler_node") {
    time_step_ = std::chrono::milliseconds(10);

    set_subscribers_and_publisher();

    tau_pub_timer_ = this->create_wall_timer(
        time_step_, std::bind(&PIDControllerNode::publish_tau, this));
    set_pid_params();
}

void PIDControllerNode::set_subscribers_and_publisher() {
    this->declare_parameter<std::string>("topics.guidance.dp");
    std::string dp_reference_topic =
        this->get_parameter("topics.guidance.dp").as_string();

    this->declare_parameter<std::string>("topics.pose");
    std::string pose_topic = this->get_parameter("topics.pose").as_string();

    this->declare_parameter<std::string>("topics.twist");
    std::string twist_topic = this->get_parameter("topics.twist").as_string();

    this->declare_parameter<std::string>("topics.killswitch");
    std::string software_kill_switch_topic =
        this->get_parameter("topics.killswitch").as_string();

    this->declare_parameter<std::string>("topics.operation_mode");
    std::string software_operation_mode_topic =
        this->get_parameter("topics.operation_mode").as_string();

    this->declare_parameter<std::string>("topics.wrench_input");
    std::string control_topic =
        this->get_parameter("topics.wrench_input").as_string();

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos_sensor_data = rclcpp::QoS(
        rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);
    killswitch_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        software_kill_switch_topic, 1,
        std::bind(&PIDControllerNode::killswitch_callback, this,
                  std::placeholders::_1));
    software_mode_sub_ = this->create_subscription<std_msgs::msg::String>(
        software_operation_mode_topic, 1,
        std::bind(&PIDControllerNode::software_mode_callback, this,
                  std::placeholders::_1));
    pose_sub_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        pose_topic, qos_sensor_data,
        std::bind(&PIDControllerNode::pose_callback, this,
                  std::placeholders::_1));
    twist_sub_ = this->create_subscription<
        geometry_msgs::msg::TwistWithCovarianceStamped>(
        twist_topic, qos_sensor_data,
        std::bind(&PIDControllerNode::twist_callback, this,
                  std::placeholders::_1));
    guidance_sub_ =
        this->create_subscription<vortex_msgs::msg::ReferenceFilter>(
            dp_reference_topic, qos_sensor_data,
            std::bind(&PIDControllerNode::guidance_callback, this,
                      std::placeholders::_1));
    tau_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
        control_topic, qos_sensor_data);
}

void PIDControllerNode::killswitch_callback(
    const std_msgs::msg::Bool::SharedPtr msg) {
    killswitch_on_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Killswitch: %s",
                killswitch_on_ ? "on" : "off");
}

void PIDControllerNode::software_mode_callback(
    const std_msgs::msg::String::SharedPtr msg) {
    software_mode_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Software mode: %s",
                software_mode_.c_str());

    if (software_mode_ == "autonomous mode") {
        eta_d_ = eta_;
    }
}

void PIDControllerNode::pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    eta_.x = msg->pose.pose.position.x;
    eta_.y = msg->pose.pose.position.y;
    eta_.z = msg->pose.pose.position.z;

    tf2::Quaternion quat;
    quat.setX(msg->pose.pose.orientation.x);
    quat.setY(msg->pose.pose.orientation.y);
    quat.setZ(msg->pose.pose.orientation.z);
    quat.setW(msg->pose.pose.orientation.w);

    tf2::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    eta_.roll = roll;
    eta_.pitch = pitch;
    eta_.yaw = yaw;
}

void PIDControllerNode::twist_callback(
    const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
    nu_.u = msg->twist.twist.linear.x;
    nu_.v = msg->twist.twist.linear.y;
    nu_.w = msg->twist.twist.linear.z;
    nu_.p = msg->twist.twist.angular.x;
    nu_.q = msg->twist.twist.angular.y;
    nu_.r = msg->twist.twist.angular.z;
}

void PIDControllerNode::publish_tau() {
    if (killswitch_on_ || software_mode_ != "autonomous mode") {
        return;
    }

    Vector6d tau = pid_controller_.calculate_tau(eta_, eta_d_, nu_, eta_dot_d_);

    geometry_msgs::msg::WrenchStamped tau_msg;
    tau_msg.header.stamp = this->now();
    tau_msg.header.frame_id = "base_link";
    tau_msg.wrench.force.x = tau(0);
    tau_msg.wrench.force.y = tau(1);
    tau_msg.wrench.force.z = tau(2);
    tau_msg.wrench.torque.x = tau(3);
    tau_msg.wrench.torque.y = tau(4);
    tau_msg.wrench.torque.z = tau(5);

    tau_pub_->publish(tau_msg);
}

void PIDControllerNode::set_pid_params() {
    this->declare_parameter<std::vector<double>>(
        "Kp", {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    this->declare_parameter<std::vector<double>>(
        "Ki", {0.1, 0.1, 0.1, 0.1, 0.1, 0.1});
    this->declare_parameter<std::vector<double>>(
        "Kd", {0.1, 0.1, 0.1, 0.1, 0.1, 0.1});

    std::vector<double> Kp_vec = this->get_parameter("Kp").as_double_array();
    std::vector<double> Ki_vec = this->get_parameter("Ki").as_double_array();
    std::vector<double> Kd_vec = this->get_parameter("Kd").as_double_array();

    Matrix6d Kp = Eigen::Map<Matrix6d>(Kp_vec.data());
    Matrix6d Ki = Eigen::Map<Matrix6d>(Ki_vec.data());
    Matrix6d Kd = Eigen::Map<Matrix6d>(Kd_vec.data());

    pid_controller_.set_kp(Kp);
    pid_controller_.set_ki(Ki);
    pid_controller_.set_kd(Kd);
}

void PIDControllerNode::guidance_callback(
    const vortex_msgs::msg::ReferenceFilter::SharedPtr msg) {
    eta_d_.x = msg->x;
    eta_d_.y = msg->y;
    eta_d_.z = msg->z;
    eta_d_.roll = msg->roll;
    eta_d_.pitch = msg->pitch;
    eta_d_.yaw = msg->yaw;
}
