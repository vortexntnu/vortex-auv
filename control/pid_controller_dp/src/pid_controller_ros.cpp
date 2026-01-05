#include <pid_controller_dp/pid_controller_ros.hpp>
#include <rclcpp/logging.hpp>
#include <variant>
#include "pid_controller_dp/pid_controller_conversions.hpp"
#include "pid_controller_dp/pid_controller_utils.hpp"
#include "pid_controller_dp/typedefs.hpp"

PIDControllerNode::PIDControllerNode() : Node("pid_controller_node") {
    time_step_ = std::chrono::milliseconds(10);

    set_subscribers_and_publisher();

    tau_pub_timer_ = this->create_wall_timer(
        time_step_, std::bind(&PIDControllerNode::publish_tau, this));
    set_pid_params();

    callback_handle_ = this->add_on_set_parameters_callback(std::bind(
        &PIDControllerNode::parametersCallback, this, std::placeholders::_1));
}

void PIDControllerNode::set_subscribers_and_publisher() {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos_sensor_data = rclcpp::QoS(
        rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

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
}

void PIDControllerNode::software_mode_callback(
    const std_msgs::msg::String::SharedPtr msg) {
    software_mode_ = msg->data;
}

void PIDControllerNode::pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    eta_ = eta_convert_from_ros_to_eigen(msg);
}

void PIDControllerNode::twist_callback(
    const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
    nu_ = nu_convert_from_ros_to_eigen(msg);
}

void PIDControllerNode::publish_tau() {
    if (killswitch_on_ || software_mode_ != "autonomous mode") {
        return;
    }

    types::Vector6d tau =
        pid_controller_.calculate_tau(eta_, eta_d_, nu_, eta_dot_d_);
    // print for debug
    RCLCPP_INFO_STREAM(this->get_logger(), "Tau: [" << tau(0) << ", " << tau(1)
                                                    << ", " << tau(2) << ", "
                                                    << tau(3) << ", " << tau(4)
                                                    << ", " << tau(5) << "]");
    // print debug
    // RCLCPP_INFO_STREAM(this->get_logger(),
    //                    "Eta error: ["
    //                        << pid_controller_.eta_error_debug.pos(0) << ", "
    //                        << pid_controller_.eta_error_debug.pos(1) << ", "
    //                        << pid_controller_.eta_error_debug.pos(2) << ", "
    //                        << pid_controller_.eta_error_debug.ori.x() << ", "
    //                        << pid_controller_.eta_error_debug.ori.y() << ", "
    //                        << pid_controller_.eta_error_debug.ori.z() << ", "
    //                        << pid_controller_.eta_error_debug.ori.w() <<
    //                        "]");
    // RCLCPP_INFO_STREAM(this->get_logger(),
    //                    "Nu desired: [" << pid_controller_.nu_d_debug(0) << ",
    //                    "
    //                                    << pid_controller_.nu_d_debug(1) << ",
    //                                    "
    //                                    << pid_controller_.nu_d_debug(2) << ",
    //                                    "
    //                                    << pid_controller_.nu_d_debug(3) << ",
    //                                    "
    //                                    << pid_controller_.nu_d_debug(4) << ",
    //                                    "
    //                                    << pid_controller_.nu_d_debug(5) <<
    //                                    "]");
    // RCLCPP_INFO_STREAM(this->get_logger(),
    //                    "Error nu: ["
    //                        << pid_controller_.error_nu_debug(0) << ", "
    //                        << pid_controller_.error_nu_debug(1) << ", "
    //                        << pid_controller_.error_nu_debug(2) << ", "
    //                        << pid_controller_.error_nu_debug(3) << ", "
    //                        << pid_controller_.error_nu_debug(4) << ", "
    //                        << pid_controller_.error_nu_debug(5) << "]");
    // RCLCPP_INFO_STREAM(this->get_logger(),
    //                    "P term: [" << pid_controller_.P_debug(0) << ", "
    //                                << pid_controller_.P_debug(1) << ", "
    //                                << pid_controller_.P_debug(2) << ", "
    //                                << pid_controller_.P_debug(3) << ", "
    //                                << pid_controller_.P_debug(4) << ", "
    //                                << pid_controller_.P_debug(5) << "]");
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Kp: [" << pid_controller_.Kp_debug(0, 0) << ", "
                               << pid_controller_.Kp_debug(0, 1) << ", "
                               << pid_controller_.Kp_debug(0, 2) << ", "
                               << pid_controller_.Kp_debug(0, 3) << ", "
                               << pid_controller_.Kp_debug(0, 4) << ", "
                               << pid_controller_.Kp_debug(0, 5) << "; "
                               << pid_controller_.Kp_debug(1, 0) << ", "
                               << pid_controller_.Kp_debug(1, 1) << ", "
                               << pid_controller_.Kp_debug(1, 2) << ", "
                               << pid_controller_.Kp_debug(1, 3) << ", "
                               << pid_controller_.Kp_debug(1, 4) << ", "
                               << pid_controller_.Kp_debug(1, 5) << "; "
                               << pid_controller_.Kp_debug(2, 0) << ", "
                               << pid_controller_.Kp_debug(2, 1) << ", "
                               << pid_controller_.Kp_debug(2, 2) << ", "
                               << pid_controller_.Kp_debug(2, 3) << ", "
                               << pid_controller_.Kp_debug(2, 4) << ", "
                               << pid_controller_.Kp_debug(2, 5) << "; "
                               << pid_controller_.Kp_debug(3, 0) << ", "
                               << pid_controller_.Kp_debug(3, 1) << ", "
                               << pid_controller_.Kp_debug(3, 2) << ", "
                               << pid_controller_.Kp_debug(3, 3) << ", "
                               << pid_controller_.Kp_debug(3, 4) << ", "
                               << pid_controller_.Kp_debug(3, 5) << "; "
                               << pid_controller_.Kp_debug(4, 0) << ", "
                               << pid_controller_.Kp_debug(4, 1) << ", "
                               << pid_controller_.Kp_debug(4, 2) << ", "
                               << pid_controller_.Kp_debug(4, 3) << ", "
                               << pid_controller_.Kp_debug(4, 4) << ", "
                               << pid_controller_.Kp_debug(4, 5) << "; "
                               << pid_controller_.Kp_debug(5, 0) << ", "
                               << pid_controller_.Kp_debug(5, 1) << ", "
                               << pid_controller_.Kp_debug(5, 2) << ", "
                               << pid_controller_.Kp_debug(5, 3) << ", "
                               << pid_controller_.Kp_debug(5, 4) << ", "
                               << pid_controller_.Kp_debug(5, 5) << "]");
    // RCLCPP_INFO_STREAM(this->get_logger(),
    //                    "I term: [" << pid_controller_.I_debug(0) << ", "
    //                                << pid_controller_.I_debug(1) << ", "
    //                                << pid_controller_.I_debug(2) << ", "
    //                                << pid_controller_.I_debug(3) << ", "
    //                                << pid_controller_.I_debug(4) << ", "
    //                                << pid_controller_.I_debug(5) << "]");
    // RCLCPP_INFO_STREAM(this->get_logger(),
    //                    "D term: [" << pid_controller_.D_debug(0) << ", "
    //                                << pid_controller_.D_debug(1) << ", "
    //                                << pid_controller_.D_debug(2) << ", "
    //                                << pid_controller_.D_debug(3) << ", "
    //                                << pid_controller_.D_debug(4) << ", "
    //                                << pid_controller_.D_debug(5) << "]");
    // RCLCPP_INFO_STREAM(this->get_logger(),
    //                    "J inv: [" << pid_controller_.J_inv_debug(0, 0) << ",
    //                    "
    //                               << pid_controller_.J_inv_debug(0, 1) << ",
    //                               "
    //                               << pid_controller_.J_inv_debug(0, 2) << ",
    //                               "
    //                               << pid_controller_.J_inv_debug(0, 3) << ",
    //                               "
    //                               << pid_controller_.J_inv_debug(0, 4) << ",
    //                               "
    //                               << pid_controller_.J_inv_debug(0, 5) << ",
    //                               "
    //                               << pid_controller_.J_inv_debug(0, 6) << ";
    //                               "
    //                               << pid_controller_.J_inv_debug(1, 0) << ",
    //                               "
    //                               << pid_controller_.J_inv_debug(1, 1) << ",
    //                               "
    //                               << pid_controller_.J_inv_debug(1, 2) << ",
    //                               "
    //                               << pid_controller_.J_inv_debug(1, 3) << ",
    //                               "
    //                               << pid_controller_.J_inv_debug(1, 4) << ",
    //                               "
    //                               << pid_controller_.J_inv_debug(1, 5) << ",
    //                               "
    //                               << pid_controller_.J_inv_debug(1, 6) << ";
    //                               "
    //                               << pid_controller_.J_inv_debug(2, 0) << ",
    //                               "
    //                               << pid_controller_.J_inv_debug(2, 1) << ",
    //                               "
    //                               << pid_controller_.J_inv_debug(2, 2) << ",
    //                               "
    //                               << pid_controller_.J_inv_debug(2, 3) << ",
    //                               "
    //                               << pid_controller_.J_inv_debug(2, 4) << ",
    //                               "
    //                               << pid_controller_.J_inv_debug(2, 5) << ",
    //                               "
    //                               << pid_controller_.J_inv_debug(2, 6) << ";
    //                               "
    //                               << pid_controller_.J_inv_debug(3, 0) << ",
    //                               "
    //                               << pid_controller_.J_inv_debug(3, 1) << ",
    //                               "
    //                               << pid_controller_.J_inv_debug(3, 2) << ",
    //                               "
    //                               << pid_controller_.J_inv_debug(3, 3) << ",
    //                               "
    //                               << pid_controller_.J_inv_debug(3, 4) << ",
    //                               "
    //                               << pid_controller_.J_inv_debug(3, 5) << ",
    //                               "
    //                               << pid_controller_.J_inv_debug(3, 6) << ";
    //                               "
    //                               << pid_controller_.J_inv_debug(4, 0) << ",
    //                               "
    //                               << pid_controller_.J_inv_debug(4, 1) << ",
    //                               "
    //                               << pid_controller_.J_inv_debug(4, 2) << ",
    //                               "
    //                               << pid_controller_.J_inv_debug(4, 3) << ",
    //                               "
    //                               << pid_controller_.J_inv_debug(4, 4) << ",
    //                               "
    //                               << pid_controller_.J_inv_debug(4, 5) << ",
    //                               "
    //                               << pid_controller_.J_inv_debug(4, 6) << ";
    //                               "
    //                               << pid_controller_.J_inv_debug(5, 0) << ",
    //                               "
    //                               << pid_controller_.J_inv_debug(5, 1) << ",
    //                               "
    //                               << pid_controller_.J_inv_debug(5, 2) << ",
    //                               "
    //                               << pid_controller_.J_inv_debug(5, 3) << ",
    //                               "
    //                               << pid_controller_.J_inv_debug(5, 4) << ",
    //                               "
    //                               << pid_controller_.J_inv_debug(5, 5) << ",
    //                               "
    //                               << pid_controller_.J_inv_debug(5, 6) <<
    //                               "]");

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
    // TODO: edit parameter declaration
    this->declare_parameter<double>("Kp_x", 1.0);
    this->declare_parameter<double>("Kp_y", 1.0);
    this->declare_parameter<double>("Kp_z", 1.0);
    this->declare_parameter<double>("Kp_roll", 1.0);
    this->declare_parameter<double>("Kp_pitch", 1.0);
    this->declare_parameter<double>("Kp_yaw", 1.0);
    this->declare_parameter<double>("Ki_x", 0.1);
    this->declare_parameter<double>("Ki_y", 0.1);
    this->declare_parameter<double>("Ki_z", 0.1);
    this->declare_parameter<double>("Ki_roll", 0.1);
    this->declare_parameter<double>("Ki_pitch", 0.1);
    this->declare_parameter<double>("Ki_yaw", 0.1);
    this->declare_parameter<double>("Kd_x", 0.1);
    this->declare_parameter<double>("Kd_y", 0.1);
    this->declare_parameter<double>("Kd_z", 0.1);
    this->declare_parameter<double>("Kd_roll", 0.1);
    this->declare_parameter<double>("Kd_pitch", 0.1);
    this->declare_parameter<double>("Kd_yaw", 0.1);

    // this->declare_parameter<std::vector<double>>(
    //     "Kp", {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    // this->declare_parameter<std::vector<double>>(
    //     "Ki", {0.1, 0.1, 0.1, 0.1, 0.1, 0.1});
    // this->declare_parameter<std::vector<double>>(
    //     "Kd", {0.1, 0.1, 0.1, 0.1, 0.1, 0.1});

    // std::vector<double> Kp_vec = this->get_parameter("Kp").as_double_array();
    // std::vector<double> Ki_vec = this->get_parameter("Ki").as_double_array();
    // std::vector<double> Kd_vec = this->get_parameter("Kd").as_double_array();

    // TODO: construct vector from parameters
    std::vector<double> Kp_vec = {
        this->get_parameter("Kp_x").as_double(),
        this->get_parameter("Kp_y").as_double(),
        this->get_parameter("Kp_z").as_double(),
        this->get_parameter("Kp_roll").as_double(),
        this->get_parameter("Kp_pitch").as_double(),
        this->get_parameter("Kp_yaw").as_double(),
    };
    std::vector<double> Ki_vec = {
        this->get_parameter("Ki_x").as_double(),
        this->get_parameter("Ki_y").as_double(),
        this->get_parameter("Ki_z").as_double(),
        this->get_parameter("Ki_roll").as_double(),
        this->get_parameter("Ki_pitch").as_double(),
        this->get_parameter("Ki_yaw").as_double(),
    };
    std::vector<double> Kd_vec = {
        this->get_parameter("Kd_x").as_double(),
        this->get_parameter("Kd_y").as_double(),
        this->get_parameter("Kd_z").as_double(),
        this->get_parameter("Kd_roll").as_double(),
        this->get_parameter("Kd_pitch").as_double(),
        this->get_parameter("Kd_yaw").as_double(),
    };

    types::Vector6d Kp_vec_eigen(Kp_vec.data());
    types::Vector6d Ki_vec_eigen(Ki_vec.data());
    types::Vector6d Kd_vec_eigen(Kd_vec.data());

    types::Matrix6d Kp_eigen = Kp_vec_eigen.asDiagonal().toDenseMatrix();
    types::Matrix6d Ki_eigen = Ki_vec_eigen.asDiagonal().toDenseMatrix();
    types::Matrix6d Kd_eigen = Kd_vec_eigen.asDiagonal().toDenseMatrix();

    // print out for debug
    // RCLCPP_INFO_STREAM(this->get_logger(),
    //                    "Kp_eigen: ["
    //                        << Kp_eigen(0, 0) << ", " << Kp_eigen(0, 1) << ",
    //                        "
    //                        << Kp_eigen(0, 2) << ", " << Kp_eigen(0, 3) << ",
    //                        "
    //                        << Kp_eigen(0, 4) << ", " << Kp_eigen(0, 5) << ";
    //                        "
    //                        << Kp_eigen(1, 0) << ", " << Kp_eigen(1, 1) << ",
    //                        "
    //                        << Kp_eigen(1, 2) << ", " << Kp_eigen(1, 3) << ",
    //                        "
    //                        << Kp_eigen(1, 4) << ", " << Kp_eigen(1, 5) << ";
    //                        "
    //                        << Kp_eigen(2, 0) << ", " << Kp_eigen(2, 1) << ",
    //                        "
    //                        << Kp_eigen(2, 2) << ", " << Kp_eigen(2, 3) << ",
    //                        "
    //                        << Kp_eigen(2, 4) << ", " << Kp_eigen(2, 5) << ";
    //                        "
    //                        << Kp_eigen(3, 0) << ", " << Kp_eigen(3, 1) << ",
    //                        "
    //                        << Kp_eigen(3, 2) << ", " << Kp_eigen(3, 3) << ",
    //                        "
    //                        << Kp_eigen(3, 4) << ", " << Kp_eigen(3, 5) << ";
    //                        "
    //                        << Kp_eigen(4, 0) << ", " << Kp_eigen(4, 1) << ",
    //                        "
    //                        << Kp_eigen(4, 2) << ", " << Kp_eigen(4, 3) << ",
    //                        "
    //                        << Kp_eigen(4, 4) << ", " << Kp_eigen(4, 5) << ";
    //                        "
    //                        << Kp_eigen(5, 0) << ", " << Kp_eigen(5, 1) << ",
    //                        "
    //                        << Kp_eigen(5, 2) << ", " << Kp_eigen(5, 3) << ",
    //                        "
    //                        << Kp_eigen(5, 4) << ", " << Kp_eigen(5, 5) <<
    //                        "]");

    pid_controller_.set_kp(Kp_eigen);
    pid_controller_.set_ki(Ki_eigen);
    pid_controller_.set_kd(Kd_eigen);
}

void PIDControllerNode::guidance_callback(
    const vortex_msgs::msg::ReferenceFilter::SharedPtr msg) {
    // Set desired position
    eta_d_.x = msg->x;
    eta_d_.y = msg->y;
    eta_d_.z = msg->z;

    // Convert desired attitude (roll, pitch, yaw) to quaternion and store
    double roll = msg->roll;
    double pitch = msg->pitch;
    double yaw = msg->yaw;

    Eigen::Quaterniond quat = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
                              Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                              Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

    eta_d_.qw = quat.w();
    eta_d_.qx = quat.x();
    eta_d_.qy = quat.y();
    eta_d_.qz = quat.z();
}

// TODO: set parameter functions
rcl_interfaces::msg::SetParametersResult PIDControllerNode::parametersCallback(
    const std::vector<rclcpp::Parameter>& parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    bool kp_x_updated = false;
    bool kp_y_updated = false;
    bool kp_z_updated = false;
    bool kp_roll_updated = false;
    bool kp_pitch_updated = false;
    bool kp_yaw_updated = false;

    bool ki_x_updated = false;
    bool ki_y_updated = false;
    bool ki_z_updated = false;
    bool ki_roll_updated = false;
    bool ki_pitch_updated = false;
    bool ki_yaw_updated = false;

    bool kd_x_updated = false;
    bool kd_y_updated = false;
    bool kd_z_updated = false;
    bool kd_roll_updated = false;
    bool kd_pitch_updated = false;
    bool kd_yaw_updated = false;

    types::Vector6d Kp_vec_eigen;
    types::Vector6d Ki_vec_eigen;
    types::Vector6d Kd_vec_eigen;

    for (const auto& param : parameters) {
        if (param.get_name() == "Kp_x") {
            Kp_vec_eigen(0) = param.as_double();
            kp_x_updated = true;
        } else if (param.get_name() == "Kp_y") {
            Kp_vec_eigen(1) = param.as_double();
            kp_y_updated = true;
        } else if (param.get_name() == "Kp_z") {
            Kp_vec_eigen(2) = param.as_double();
            kp_z_updated = true;
        } else if (param.get_name() == "Kp_roll") {
            Kp_vec_eigen(3) = param.as_double();
            kp_roll_updated = true;
        } else if (param.get_name() == "Kp_pitch") {
            Kp_vec_eigen(4) = param.as_double();
            kp_pitch_updated = true;
        } else if (param.get_name() == "Kp_yaw") {
            Kp_vec_eigen(5) = param.as_double();
            kp_yaw_updated = true;
        } else if (param.get_name() == "Ki_x") {
            Ki_vec_eigen(0) = param.as_double();
            ki_x_updated = true;
        } else if (param.get_name() == "Ki_y") {
            Ki_vec_eigen(1) = param.as_double();
            ki_y_updated = true;
        } else if (param.get_name() == "Ki_z") {
            Ki_vec_eigen(2) = param.as_double();
            ki_z_updated = true;
        } else if (param.get_name() == "Ki_roll") {
            Ki_vec_eigen(3) = param.as_double();
            ki_roll_updated = true;
        } else if (param.get_name() == "Ki_pitch") {
            Ki_vec_eigen(4) = param.as_double();
            ki_pitch_updated = true;
        } else if (param.get_name() == "Ki_yaw") {
            Ki_vec_eigen(5) = param.as_double();
            ki_yaw_updated = true;
        } else if (param.get_name() == "Kd_x") {
            Kd_vec_eigen(0) = param.as_double();
            kd_x_updated = true;
        } else if (param.get_name() == "Kd_y") {
            Kd_vec_eigen(1) = param.as_double();
            kd_y_updated = true;
        } else if (param.get_name() == "Kd_z") {
            Kd_vec_eigen(2) = param.as_double();
            kd_z_updated = true;
        } else if (param.get_name() == "Kd_roll") {
            Kd_vec_eigen(3) = param.as_double();
            kd_roll_updated = true;
        } else if (param.get_name() == "Kd_pitch") {
            Kd_vec_eigen(4) = param.as_double();
            kd_pitch_updated = true;
        } else if (param.get_name() == "Kd_yaw") {
            Kd_vec_eigen(5) = param.as_double();
            kd_yaw_updated = true;
        }
    }

    // Only set the gains if the parameter update was successful
    if (result.successful) {
        if (kp_x_updated || kp_y_updated || kp_z_updated || kp_roll_updated ||
            kp_pitch_updated || kp_yaw_updated) {
            types::Matrix6d Kp_eigen =
                Kp_vec_eigen.asDiagonal().toDenseMatrix();
            pid_controller_.set_kp(Kp_eigen);
        }
        if (ki_x_updated || ki_y_updated || ki_z_updated || ki_roll_updated ||
            ki_pitch_updated || ki_yaw_updated) {
            types::Matrix6d Ki_eigen =
                Ki_vec_eigen.asDiagonal().toDenseMatrix();
            pid_controller_.set_ki(Ki_eigen);
        }
        if (kd_x_updated || kd_y_updated || kd_z_updated || kd_roll_updated ||
            kd_pitch_updated || kd_yaw_updated) {
            types::Matrix6d Kd_eigen =
                Kd_vec_eigen.asDiagonal().toDenseMatrix();
            pid_controller_.set_kd(Kd_eigen);
        }
    }

    // print
    for (const auto& param : parameters) {
        RCLCPP_INFO(this->get_logger(), "%s", param.get_name().c_str());
        RCLCPP_INFO(this->get_logger(), "%s", param.get_type_name().c_str());
        RCLCPP_INFO(this->get_logger(), "%s", param.value_to_string().c_str());
    }
    return result;
}
