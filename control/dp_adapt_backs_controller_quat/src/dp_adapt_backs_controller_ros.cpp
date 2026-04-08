#include "dp_adapt_backs_controller_quat/dp_adapt_backs_controller_ros.hpp"
#include <spdlog/spdlog.h>
#include <rclcpp_components/register_node_macro.hpp>
#include <string>
#include <vortex/utils/math.hpp>
#include <vortex/utils/ros/qos_profiles.hpp>
#include <vortex/utils/ros/ros_conversions.hpp>
#include <vortex/utils/types.hpp>
#include "dp_adapt_backs_controller_quat/dp_adapt_backs_controller.hpp"
#include "dp_adapt_backs_controller_quat/dp_adapt_backs_controller_utils.hpp"
#include "dp_adapt_backs_controller_quat/typedefs.hpp"

constexpr std::string_view start_message = R"(
 ██████╗ ██╗   ██╗ █████╗ ████████╗███████╗██████╗ ███╗   ██╗██╗ ██████╗ ███╗   ██╗    ██████╗ ██████╗
██╔═══██╗██║   ██║██╔══██╗╚══██╔══╝██╔════╝██╔══██╗████╗  ██║██║██╔═══██╗████╗  ██║    ██╔══██╗██╔══██╗
██║   ██║██║   ██║███████║   ██║   █████╗  ██████╔╝██╔██╗ ██║██║██║   ██║██╔██╗ ██║    ██║  ██║██████╔╝
██║▄▄ ██║██║   ██║██╔══██║   ██║   ██╔══╝  ██╔══██╗██║╚██╗██║██║██║   ██║██║╚██╗██║    ██║  ██║██╔═══╝
╚██████╔╝╚██████╔╝██║  ██║   ██║   ███████╗██║  ██║██║ ╚████║██║╚██████╔╝██║ ╚████║    ██████╔╝██║
 ╚══▀▀═╝  ╚═════╝ ╚═╝  ╚═╝   ╚═╝   ╚══════╝╚═╝  ╚═╝╚═╝  ╚═══╝╚═╝ ╚═════╝ ╚═╝  ╚═══╝    ╚═════╝ ╚═╝
)";

namespace vortex::control {

DPAdaptBacksControllerNode::DPAdaptBacksControllerNode(
    const rclcpp::NodeOptions& options)
    : Node("dp_adapt_backs_controller_node", options) {
    this->declare_parameter<int>("time_step_ms");
    time_step_ms = std::chrono::milliseconds(
        static_cast<int>(this->get_parameter("time_step_ms").as_int()));

    set_subscribers_and_publisher();
    initialize_operation_mode();

    tau_pub_timer_ =
        this->create_wall_timer(time_step_ms, [this]() { publish_tau(); });
    set_adap_params();

    spdlog::info(start_message);
}

void DPAdaptBacksControllerNode::set_subscribers_and_publisher() {
    const auto qos_sensor_data{
        vortex::utils::qos_profiles::sensor_data_profile(1)};
    const auto qos_reliable{vortex::utils::qos_profiles::reliable_profile(1)};

    this->declare_parameter<std::string>("topics.guidance.dp_quat");
    std::string dp_reference_topic =
        this->get_parameter("topics.guidance.dp_quat").as_string();
    guidance_sub_ =
        this->create_subscription<vortex_msgs::msg::ReferenceFilterQuat>(
            dp_reference_topic, qos_sensor_data,
            [this](const vortex_msgs::msg::ReferenceFilterQuat::SharedPtr msg) {
                guidance_callback(msg);
            });

    this->declare_parameter<std::string>("topics.pose");
    std::string pose_topic = this->get_parameter("topics.pose").as_string();
    pose_sub_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        pose_topic, qos_sensor_data,
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr
                   msg) { pose_callback(msg); });

    this->declare_parameter<std::string>("topics.twist");
    std::string twist_topic = this->get_parameter("topics.twist").as_string();
    twist_sub_ = this->create_subscription<
        geometry_msgs::msg::TwistWithCovarianceStamped>(
        twist_topic, qos_sensor_data,
        [this](const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr
                   msg) { twist_callback(msg); });

    this->declare_parameter<std::string>("topics.killswitch");
    std::string software_kill_switch_topic =
        this->get_parameter("topics.killswitch").as_string();
    killswitch_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        software_kill_switch_topic, qos_reliable,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            killswitch_callback(msg);
        });

    this->declare_parameter<std::string>("topics.operation_mode");
    std::string software_operation_mode_topic =
        this->get_parameter("topics.operation_mode").as_string();
    operation_mode_sub_ =
        this->create_subscription<vortex_msgs::msg::OperationMode>(
            software_operation_mode_topic, qos_reliable,
            [this](const vortex_msgs::msg::OperationMode::SharedPtr msg) {
                operation_mode_callback(msg);
            });

    this->declare_parameter<std::string>("topics.wrench_input");
    std::string control_topic =
        this->get_parameter("topics.wrench_input").as_string();
    tau_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
        control_topic, qos_sensor_data);
}

void DPAdaptBacksControllerNode::initialize_operation_mode() {
    this->declare_parameter<std::string>("services.get_operation_mode");
    std::string get_operation_mode_service =
        this->get_parameter("services.get_operation_mode").as_string();

    get_operation_mode_client_ =
        this->create_client<vortex_msgs::srv::GetOperationMode>(
            get_operation_mode_service);

    while (!get_operation_mode_client_->wait_for_service(
        std::chrono::seconds(1))) {
        spdlog::warn("Waiting for GetOperationMode service to be available...");
    }

    auto request =
        std::make_shared<vortex_msgs::srv::GetOperationMode::Request>();
    get_operation_mode_client_->async_send_request(
        request,
        [this](rclcpp::Client<vortex_msgs::srv::GetOperationMode>::SharedFuture
                   future) {
            try {
                auto response = future.get();
                operation_mode_ =
                    vortex::utils::ros_conversions::convert_from_ros(
                        response->current_operation_mode);
                killswitch_on_ = response->killswitch_status;
                spdlog::info(
                    "Initial operation mode: {} | Killswitch status: {}",
                    vortex::utils::types::mode_to_string(operation_mode_),
                    killswitch_on_ ? "on" : "off");
            } catch (const std::exception& e) {
                spdlog::error("Failed to get initial operation mode: {}",
                              e.what());
                killswitch_on_ = true;
            }
        });
}

void DPAdaptBacksControllerNode::killswitch_callback(
    const std_msgs::msg::Bool::SharedPtr msg) {
    killswitch_on_ = msg->data;
    dp_adapt_backs_controller_->reset_adap_param();
    dp_adapt_backs_controller_->reset_d_est();
    spdlog::info("Killswitch: {}", killswitch_on_ ? "on" : "off");
}

void DPAdaptBacksControllerNode::operation_mode_callback(
    const vortex_msgs::msg::OperationMode::SharedPtr msg) {
    operation_mode_ = vortex::utils::ros_conversions::convert_from_ros(*msg);
    spdlog::info("Operation mode: {}",
                 vortex::utils::types::mode_to_string(operation_mode_));

    if (operation_mode_ == vortex::utils::types::Mode::autonomous ||
        operation_mode_ == vortex::utils::types::Mode::reference) {
        pose_d_ = pose_;
    }
}

void DPAdaptBacksControllerNode::pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    pose_ = vortex::utils::ros_conversions::ros_pose_to_pose(msg->pose.pose);
}

void DPAdaptBacksControllerNode::twist_callback(
    const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
    twist_ =
        vortex::utils::ros_conversions::ros_twist_to_twist(msg->twist.twist);
}

void DPAdaptBacksControllerNode::set_adap_params() {
    this->declare_parameter<std::vector<double>>("adapt_gain");
    this->declare_parameter<std::vector<double>>("d_gain");
    this->declare_parameter<std::vector<double>>("K1");
    this->declare_parameter<std::vector<double>>("K2");
    this->declare_parameter<std::vector<double>>("r_b_bg");
    this->declare_parameter<std::vector<double>>("physical.mass_matrix");
    this->declare_parameter<std::vector<double>>("physical.center_of_mass");
    this->declare_parameter<std::vector<double>>(
        "propulsion.thrusters.thruster_force_direction");
    this->declare_parameter<std::vector<double>>(
        "propulsion.thrusters.thruster_position");
    this->declare_parameter<int>("propulsion.thrusters.num");
    this->declare_parameter<int>("propulsion.dimensions.num");
    this->declare_parameter<double>(
        "propulsion.thrusters.constraints.min_force");
    this->declare_parameter<double>(
        "propulsion.thrusters.constraints.max_force");
    this->declare_parameter<double>("singularity_tolerance");
    this->declare_parameter<double>("adapt_param_max");
    this->declare_parameter<double>("d_est_max");

    std::vector<double> adapt_param_vec =
        this->get_parameter("adapt_gain").as_double_array();
    std::vector<double> d_gain_vec =
        this->get_parameter("d_gain").as_double_array();
    std::vector<double> K1_vec = this->get_parameter("K1").as_double_array();
    std::vector<double> K2_vec = this->get_parameter("K2").as_double_array();
    std::vector<double> r_b_bg_vec =
        this->get_parameter("r_b_bg").as_double_array();
    std::vector<double> mass_intertia_matrix_vec =
        this->get_parameter("physical.mass_matrix").as_double_array();

    Eigen::Vector12d adapt_param_eigen =
        Eigen::Map<Eigen::Vector12d>(adapt_param_vec.data());
    Eigen::Vector6d d_gain_eigen =
        Eigen::Map<Eigen::Vector6d>(d_gain_vec.data());
    Eigen::Vector6d K1_eigen = Eigen::Map<Eigen::Vector6d>(K1_vec.data());
    Eigen::Vector6d K2_eigen = Eigen::Map<Eigen::Vector6d>(K2_vec.data());
    Eigen::Vector3d r_b_bg_eigen =
        Eigen::Map<Eigen::Vector3d>(r_b_bg_vec.data());
    Eigen::Matrix6d mass_intertia_matrix =
        Eigen::Map<Eigen::Matrix6d>(mass_intertia_matrix_vec.data());

    double mass = mass_intertia_matrix(0, 0);
    Eigen::Vector3d inertia_matrix_body_eigen(mass_intertia_matrix(3, 3),
                                              mass_intertia_matrix(4, 4),
                                              mass_intertia_matrix(5, 5));

    // Compute per-DOF max wrench from the thruster configuration
    int num_thrusters =
        this->get_parameter("propulsion.thrusters.num").as_int();
    int num_dims = this->get_parameter("propulsion.dimensions.num").as_int();
    double min_force =
        this->get_parameter("propulsion.thrusters.constraints.min_force")
            .as_double();
    double max_force =
        this->get_parameter("propulsion.thrusters.constraints.max_force")
            .as_double();

    Eigen::Vector3d center_of_mass = Eigen::Map<const Eigen::Vector3d>(
        this->get_parameter("physical.center_of_mass")
            .as_double_array()
            .data());

    auto dir_vec =
        this->get_parameter("propulsion.thrusters.thruster_force_direction")
            .as_double_array();
    auto pos_vec = this->get_parameter("propulsion.thrusters.thruster_position")
                       .as_double_array();

    Eigen::MatrixXd thruster_dir =
        Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                                       Eigen::RowMajor>>(
            dir_vec.data(), num_dims, num_thrusters);
    Eigen::MatrixXd thruster_pos =
        Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                                       Eigen::RowMajor>>(
            pos_vec.data(), num_dims, num_thrusters);

    const Eigen::MatrixXd T =
        vortex::utils::math::build_thrust_configuration_matrix(
            thruster_dir, thruster_pos, center_of_mass);
    const Eigen::VectorXd u_min =
        Eigen::VectorXd::Constant(num_thrusters, min_force);
    const Eigen::VectorXd u_max =
        Eigen::VectorXd::Constant(num_thrusters, max_force);
    const Eigen::Vector6d tau_max =
        vortex::utils::math::calculate_valid_thrust_region_polyhedron(T, u_min,
                                                                      u_max);

    DPAdaptParams dp_adapt_params = DPAdaptParams{
        .adapt_param = adapt_param_eigen,
        .d_gain = d_gain_eigen,
        .K1 = K1_eigen,
        .K2 = K2_eigen,
        .r_b_bg = r_b_bg_eigen,
        .inertia_matrix_body = inertia_matrix_body_eigen,
        .mass_intertia_matrix = mass_intertia_matrix,
        .tau_max = tau_max,
        .mass = mass,
        .time_step_s = static_cast<double>(time_step_ms.count()) / 1000.0,
        .singularity_tolerance =
            this->get_parameter("singularity_tolerance").as_double(),
        .adapt_param_max = this->get_parameter("adapt_param_max").as_double(),
        .d_est_max = this->get_parameter("d_est_max").as_double(),
    };

    dp_adapt_backs_controller_ =
        std::make_unique<DPAdaptBacksController>(dp_adapt_params);
}

void DPAdaptBacksControllerNode::publish_tau() {
    if (killswitch_on_ ||
        operation_mode_ == vortex::utils::types::Mode::manual) {
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
    tau_msg.wrench.torque.x = tau(3);
    tau_msg.wrench.torque.y = tau(4);
    tau_msg.wrench.torque.z = tau(5);

    tau_pub_->publish(tau_msg);
}

void DPAdaptBacksControllerNode::guidance_callback(
    const vortex_msgs::msg::ReferenceFilterQuat::SharedPtr msg) {
    pose_d_ =
        vortex::utils::ros_conversions::reference_filter_quat_to_pose(*msg);
}

RCLCPP_COMPONENTS_REGISTER_NODE(DPAdaptBacksControllerNode)

}  // namespace vortex::control

// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⣀⣠⡤⣦⢦⠖⡶⠲⢦⣤⢤⣤⣤⣀⢀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣠⡴⢶⠛⡍⢎⠱⣈⠒⢌⡘⠰⠉⢆⠰⢉⠔⠢⡉⢝⢫⠳⢦⣄⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣠⡶⢏⠳⡘⠢⢉⠔⡈⠔⠠⠈⠄⠠⠁⠌⡀⠂⠌⠠⠁⠌⡐⢂⠉⢆⠩⡝⢳⣦⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣴⡞⢫⠔⡉⠆⡁⠢⢁⣂⡐⡈⠄⢁⠈⠄⣁⡤⠄⣁⣠⠁⡈⠄⠐⠠⠉⠠⠁⡌⢡⠊⡝⢷⣄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⣠⡾⢣⠎⡡⢊⠴⣅⣶⣽⣷⣾⣿⣶⠌⠀⠄⢺⢾⣿⣿⣿⣷⣾⣿⣤⣧⡰⠈⠠⢁⠐⢂⠑⠌⡸⠸⣷⣤⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⣸⢯⣑⠣⣌⠱⢨⣿⣿⣿⣿⡿⠟⠋⠁⡀⠌⠐⠠⢁⠚⡹⢛⡿⢿⣿⣿⣿⠯⢀⠁⠂⠌⠠⢈⠂⣁⠣⡘⠽⣦⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⢴⡟⡥⢊⠵⣠⠣⢍⠺⢟⠩⣁⠒⠨⢀⠂⠀⠄⠡⡁⢂⠠⠀⠄⢠⠉⠄⡊⢄⠂⡄⠊⠄⢃⠤⢁⠂⢄⠂⠥⡙⣚⢷⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⢀⡿⡜⠴⣉⠖⣡⢚⣬⢳⡉⠖⣀⠃⣁⠢⠐⡈⠠⢁⠒⡄⢂⠡⠊⠄⡘⠤⠑⡌⢒⠤⢃⠜⣀⠂⢆⠘⡠⠘⡀⢆⢡⢻⣧⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⢸⣛⣌⠳⡌⢎⡵⣫⣎⠧⡘⠡⠀⠂⢀⠂⠱⢠⢁⠊⡴⣘⠢⠁⠌⠀⠐⠠⢁⠘⡄⠫⡜⠲⣄⠩⢄⠢⠁⠆⠡⠌⡂⢎⡽⣆⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⢸⡷⢌⠳⡌⢧⣿⢿⡜⣢⣱⣤⡧⣐⣢⡀⢃⢢⠁⢎⡶⣡⢆⣵⠴⠧⢦⣁⣂⠰⣈⠱⣘⠳⣬⡓⢌⠢⡉⢌⠡⡘⢄⢣⢚⣿⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⢹⡞⣌⠳⡌⢧⣿⣿⣽⢷⣫⣔⡠⢀⢀⠈⠉⢲⣭⣰⣟⣯⣹⣔⣢⢡⠀⡀⠌⠙⠞⣳⠿⢷⣷⣝⠦⡑⠨⠄⡃⠔⡈⠦⣙⡾⡆⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⢸⡿⢤⠓⣌⢣⢻⣹⣿⣻⣿⣿⣿⣶⠢⣌⢡⠂⡴⢻⡿⣷⣿⣿⣿⣷⣳⠴⡈⡜⡰⣄⢻⣜⣿⢳⡡⢊⡑⢌⠰⠡⡘⡰⣡⢿⡇⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠸⣟⢦⢋⡔⢣⡙⣷⡻⣿⣿⣿⣿⣿⣷⣎⢦⣙⢶⢏⠻⣿⣿⣿⣿⣿⡿⢧⡹⣴⢳⣽⣳⡿⢡⠗⡠⢃⠔⡈⢆⠱⣐⠱⡬⢿⡇⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⢻⣎⠖⡬⢑⡜⢲⢻⣮⠻⣿⣿⣻⣷⣾⡻⢞⠧⢎⠛⣮⣛⠿⣽⣻⣽⣯⣿⣾⣿⠿⢋⡴⢋⡒⢡⠊⡔⣁⠊⡔⠤⣛⠼⣿⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠘⢯⣞⠰⡡⠎⢥⢋⡞⡿⢶⡞⣭⡝⣲⡙⢎⡱⢊⡱⠄⣍⠛⢦⢋⠭⣉⡍⣡⠔⣎⠣⡜⡡⠘⡄⢣⠐⡄⢣⠘⢦⢭⣿⡟⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⣠⣤⡸⣿⢢⡑⠎⣆⢣⠚⣭⢣⡛⡴⡙⢆⠹⢠⠑⠢⢄⠃⠄⣉⠂⠍⠒⡡⠜⠤⢋⠔⡃⢆⠡⢃⡘⢄⠣⡘⢤⢋⡼⣺⣿⠉⣀⣤⣄⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⣤⡗⠀⣉⣻⣧⣙⠲⡐⢎⡱⢂⠇⣎⠱⢡⠊Life is pain,⡈⢂⠡⠌⡐⡈⠤⢁⠰⡈⢆⠱⣊⠼⣼⣿⣿⣛⣉⠐⠾⣄⠀⠀⠀⠀
// ⠀⠀⠀⡼⠃⠀⠀⠙⣿⡘⢿⣵⠿⡼⣴⣩⣚Existence is meaningless⢢⣑⣎⡱⢬⣻⣿⠇⣸⡿⠁⠀⠀⠙⣆⠀⠀⠀
// ⠀⠀⣰⡗⠀⠀⠀⠀⠘⣧⡀⠙⠻⣷⣾⣭⣿⣹⢏⡻⢩⢛⠛⡛⢛⠻⠛⠟⡛⠟⢛⠛⡛⠹⢛⠛⣙⡋⣏⣙⣩⣭⣭⣴⣼⣿⠟⠁⠈⣿⠁⠀⠀⠀⠀⢾⡄⠀⠀
// ⠀⢠⠻⠀⢀⣠⣄⠀⠀⣻⠀⠀⠀⠀⠛⠿⣿⣿⣿⣷⣷⣮⣵⣌⣦⡱⣌⣲⡰⣌⣦⣱⣬⣷⣾⣿⣿⣿⡿⣿⢿⣿⣿⣿⠛⠁⠀⢀⢸⡇⠀⠀⣀⣀⠀⠘⡿⡀⠀
// ⠀⣧⠇⠀⣴⠟⠈⠀⠀⣿⡄⠀⠀⠀⠀⠀⠈⠙⠻⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⢿⣻⢯⣷⣯⣿⣽⡿⠟⠉⠀⠀⠀⠀⠀⣘⣿⠀⢸⠉⠿⣄⠀⠰⣡⠀
// ⠸⡌⠀⢹⡇⠀⠀⣏⠀⣽⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠙⠛⠛⠿⠿⠿⢽⢿⡾⠷⠿⠿⠿⠟⠛⠋⠉⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣇⠀⠨⠀⠀⢹⠇⠀⢏⠂
// ⠀⡇⠀⣻⡆⠀⠀⠉⠒⠋⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠓⠒⠃⠀⠀⢸⣟⠀⣸⠀
// ⠀⠣⠴⠞⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠘⠷⠄⠆⠀
