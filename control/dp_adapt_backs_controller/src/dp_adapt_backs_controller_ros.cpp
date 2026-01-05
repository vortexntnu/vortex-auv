#include "dp_adapt_backs_controller/dp_adapt_backs_controller_ros.hpp"
#include <spdlog/spdlog.h>
#include <rclcpp_components/register_node_macro.hpp>
#include <string>
#include <vortex/utils/math.hpp>
#include <vortex/utils/ros/qos_profiles.hpp>
#include <vortex/utils/types.hpp>
#include "dp_adapt_backs_controller/dp_adapt_backs_controller_utils.hpp"
#include "dp_adapt_backs_controller/typedefs.hpp"

constexpr std::string_view start_message = R"(
  ____  ____     ____            _             _ _
 |  _ \|  _ \   / ___|___  _ __ | |_ _ __ ___ | | | ___ _ __
 | | | | |_) | | |   / _ \| '_ \| __| '__/ _ \| | |/ _ \ '__|
 | |_| |  __/  | |__| (_) | | | | |_| | | (_) | | |  __/ |
 |____/|_|      \____\___/|_| |_|\__|_|  \___/|_|_|\___|_|

)";

namespace vortex::control {

DPAdaptBacksControllerNode::DPAdaptBacksControllerNode(
    const rclcpp::NodeOptions& options)
    : Node("dp_adapt_backs_controller_node", options) {
    time_step_ = std::chrono::milliseconds(10);

    set_subscribers_and_publisher();

    tau_pub_timer_ = this->create_wall_timer(
        time_step_, std::bind(&DPAdaptBacksControllerNode::publish_tau, this));
    set_adap_params();

    spdlog::info(start_message);
}

void DPAdaptBacksControllerNode::set_subscribers_and_publisher() {
    const auto qos_sensor_data{
        vortex::utils::qos_profiles::sensor_data_profile(1)};
    const auto qos_reliable{vortex::utils::qos_profiles::reliable_profile(1)};

    this->declare_parameter<std::string>("topics.guidance.dp");
    std::string dp_reference_topic =
        this->get_parameter("topics.guidance.dp").as_string();
    guidance_sub_ =
        this->create_subscription<vortex_msgs::msg::ReferenceFilter>(
            dp_reference_topic, qos_sensor_data,
            std::bind(&DPAdaptBacksControllerNode::guidance_callback, this,
                      std::placeholders::_1));

    this->declare_parameter<std::string>("topics.pose");
    std::string pose_topic = this->get_parameter("topics.pose").as_string();
    pose_sub_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        pose_topic, qos_sensor_data,
        std::bind(&DPAdaptBacksControllerNode::pose_callback, this,
                  std::placeholders::_1));

    this->declare_parameter<std::string>("topics.twist");
    std::string twist_topic = this->get_parameter("topics.twist").as_string();
    twist_sub_ = this->create_subscription<
        geometry_msgs::msg::TwistWithCovarianceStamped>(
        twist_topic, qos_sensor_data,
        std::bind(&DPAdaptBacksControllerNode::twist_callback, this,
                  std::placeholders::_1));

    this->declare_parameter<std::string>("topics.killswitch");
    std::string software_kill_switch_topic =
        this->get_parameter("topics.killswitch").as_string();
    killswitch_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        software_kill_switch_topic, qos_reliable,
        std::bind(&DPAdaptBacksControllerNode::killswitch_callback, this,
                  std::placeholders::_1));

    this->declare_parameter<std::string>("topics.operation_mode");
    std::string software_operation_mode_topic =
        this->get_parameter("topics.operation_mode").as_string();
    software_mode_sub_ = this->create_subscription<std_msgs::msg::String>(
        software_operation_mode_topic, qos_reliable,
        std::bind(&DPAdaptBacksControllerNode::software_mode_callback, this,
                  std::placeholders::_1));

    this->declare_parameter<std::string>("topics.wrench_input");
    std::string control_topic =
        this->get_parameter("topics.wrench_input").as_string();
    tau_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
        control_topic, qos_sensor_data);
}

void DPAdaptBacksControllerNode::killswitch_callback(
    const std_msgs::msg::Bool::SharedPtr msg) {
    killswitch_on_ = msg->data;
    dp_adapt_backs_controller_->reset_adap_param();
    dp_adapt_backs_controller_->reset_d_est();
    spdlog::info("Killswitch: {}", killswitch_on_ ? "on" : "off");
}

void DPAdaptBacksControllerNode::software_mode_callback(
    const std_msgs::msg::String::SharedPtr msg) {
    software_mode_ = msg->data;
    spdlog::info("Software mode: {}", software_mode_);

    if (software_mode_ == "autonomous mode") {
        pose_d_ = pose_;
    }
}

void DPAdaptBacksControllerNode::pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    pose_.x = msg->pose.pose.position.x;
    pose_.y = msg->pose.pose.position.y;
    pose_.z = msg->pose.pose.position.z;
    const auto& o = msg->pose.pose.orientation;
    Eigen::Quaterniond q(o.w, o.x, o.y, o.z);
    Eigen::Vector3d euler_angles = vortex::utils::math::quat_to_euler(q);
    pose_.roll = euler_angles(0);
    pose_.pitch = euler_angles(1);
    pose_.yaw = euler_angles(2);
}

void DPAdaptBacksControllerNode::twist_callback(
    const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
    twist_.u = msg->twist.twist.linear.x;
    twist_.v = msg->twist.twist.linear.y;
    twist_.w = msg->twist.twist.linear.z;
    twist_.p = msg->twist.twist.angular.x;
    twist_.q = msg->twist.twist.angular.y;
    twist_.r = msg->twist.twist.angular.z;
}

void DPAdaptBacksControllerNode::set_adap_params() {
    this->declare_parameter<std::vector<double>>("adapt_gain");
    this->declare_parameter<std::vector<double>>("d_gain");
    this->declare_parameter<std::vector<double>>("K1");
    this->declare_parameter<std::vector<double>>("K2");
    this->declare_parameter<std::vector<double>>("r_b_bg");
    this->declare_parameter<std::vector<double>>("inertia_matrix");
    this->declare_parameter<std::vector<double>>("mass_matrix");
    this->declare_parameter<double>("m");

    std::vector<double> adapt_param_vec =
        this->get_parameter("adapt_gain").as_double_array();
    std::vector<double> d_gain_vec =
        this->get_parameter("d_gain").as_double_array();
    std::vector<double> K1_vec = this->get_parameter("K1").as_double_array();
    std::vector<double> K2_vec = this->get_parameter("K2").as_double_array();
    std::vector<double> r_b_bg_vec =
        this->get_parameter("r_b_bg").as_double_array();
    std::vector<double> I_b_vec =
        this->get_parameter("inertia_matrix").as_double_array();
    std::vector<double> mass_matrix_vec =
        this->get_parameter("mass_matrix").as_double_array();

    double mass{this->get_parameter("m").as_double()};

    Eigen::Vector12d adapt_param_eigen =
        Eigen::Map<Eigen::Vector12d>(adapt_param_vec.data());
    Eigen::Vector6d d_gain_eigen =
        Eigen::Map<Eigen::Vector6d>(d_gain_vec.data());
    Eigen::Vector6d K1_eigen = Eigen::Map<Eigen::Vector6d>(K1_vec.data());
    Eigen::Vector6d K2_eigen = Eigen::Map<Eigen::Vector6d>(K2_vec.data());
    Eigen::Vector3d r_b_bg_eigen =
        Eigen::Map<Eigen::Vector3d>(r_b_bg_vec.data());
    Eigen::Vector3d I_b_eigen = Eigen::Map<Eigen::Vector3d>(I_b_vec.data());
    Eigen::Matrix6d mass_matrix =
        Eigen::Map<Eigen::Matrix6d>(mass_matrix_vec.data());

    DPAdaptParams dp_adapt_params;
    dp_adapt_params.adapt_param = adapt_param_eigen;
    dp_adapt_params.d_gain = d_gain_eigen;
    dp_adapt_params.K1 = K1_eigen;
    dp_adapt_params.K2 = K2_eigen;
    dp_adapt_params.r_b_bg = r_b_bg_eigen;
    dp_adapt_params.I_b = I_b_eigen;
    dp_adapt_params.mass_matrix = mass_matrix;
    dp_adapt_params.mass = mass;

    dp_adapt_backs_controller_ =
        std::make_unique<DPAdaptBacksController>(dp_adapt_params);
    ;
}

void DPAdaptBacksControllerNode::publish_tau() {
    if (killswitch_on_ || software_mode_ != "autonomous mode") {
        return;
    }

    Eigen::Vector6d tau =
        dp_adapt_backs_controller_->calculate_tau(pose_, pose_d_, twist_);

    geometry_msgs::msg::WrenchStamped tau_msg;
    tau_msg.header.stamp = this->now();
    tau_msg.header.frame_id = "base_link";
    tau_msg.wrench.force.x = tau(0);
    tau_msg.wrench.force.y = tau(1);
    tau_msg.wrench.force.z = tau(2);
    // tau_msg.wrench.torque.x = tau(3); commented out since roll control is not
    // needed and causes minor instability, if needed uncomment
    tau_msg.wrench.torque.y = tau(4);
    tau_msg.wrench.torque.z = tau(5);

    tau_pub_->publish(tau_msg);
}

void DPAdaptBacksControllerNode::guidance_callback(
    const vortex_msgs::msg::ReferenceFilter::SharedPtr msg) {
    pose_d_.x = msg->x;
    pose_d_.y = msg->y;
    pose_d_.z = msg->z;
    pose_d_.roll = msg->roll;
    pose_d_.pitch = msg->pitch;
    pose_d_.yaw = msg->yaw;
}

RCLCPP_COMPONENTS_REGISTER_NODE(DPAdaptBacksControllerNode)

}  // namespace vortex::control
