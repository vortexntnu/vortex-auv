#include "landmark_server/landmark_server_ros.hpp"
#include <spdlog/spdlog.h>
#include <rclcpp_action/create_client.hpp>
#include <rclcpp_action/create_server.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <vortex/utils/ros/qos_profiles.hpp>
#include <vortex/utils/ros/ros_conversions.hpp>
#include <vortex/utils/ros/ros_transforms.hpp>

namespace vortex::mission {

LandmarkServerNode::LandmarkServerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("landmark_server_node", options) {
    setup_ros_communicators();

    enu_ned_rotation_ = this->declare_parameter<bool>("enu_ned_rotation");
    int timer_rate_ms = this->declare_parameter<int>("timer_rate_ms");
    target_frame_ = this->declare_parameter<std::string>("target_frame");
    filter_dt_seconds_ = static_cast<double>(timer_rate_ms) / 1000;
}

void LandmarkServerNode::setup_ros_communicators() {
    create_reference_publisher();
    create_pose_subscription();
    create_polling_action_server();
    create_convergence_action_server();
    create_reference_action_client();
}

void LandmarkServerNode::create_reference_publisher() {
    std::string reference_pose_topic =
        this->declare_parameter<std::string>("topics.reference_pose");
    auto qos_sensor_profile =
        vortex::utils::qos_profiles::sensor_data_profile(5);
    reference_pose_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>(
            reference_pose_topic, qos_sensor_profile);
}

void LandmarkServerNode::create_pose_subscription() {
    std::string landmark_topic =
        this->declare_parameter<std::string>("topics.landmarks");
    auto qos_sensor_profile =
        vortex::utils::qos_profiles::sensor_data_profile(5);
    auto sub = std::make_shared<
        message_filters::Subscriber<vortex_msgs::msg::LandmarkArray>>(
        this, landmark_topic, qos_sensor_profile.get_rmw_qos_profile());

    auto filter = std::make_shared<
        tf2_ros::MessageFilter<vortex_msgs::msg::LandmarkArray>>(
        *sub, *tf2_buffer_, target_frame_, 10,
        this->get_node_logging_interface(), this->get_node_clock_interface());

    filter->registerCallback(
        [this](const typename vortex_msgs::msg::LandmarkArray::ConstSharedPtr
                   msg) {
            vortex_msgs::msg::LandmarkArray pose_tf;
            try {
                vortex::utils::ros_transforms::transform_pose(
                    *tf2_buffer_, *msg, target_frame_, pose_tf);
            } catch (const tf2::TransformException& ex) {
                RCLCPP_WARN(this->get_logger(),
                            "TF transform failed from '%s' to '%s': %s",
                            msg->header.frame_id.c_str(), target_frame_.c_str(),
                            ex.what());
                return;
            }

            this->measurements_ = ros_msg_to_landmarks(pose_tf);
            if (enu_ned_rotation_) {
                std::ranges::for_each(this->measurements_, [](auto& m) {
                    m.pose.set_ori(vortex::utils::math::enu_ned_rotation(
                        m.pose.ori_quaternion()));
                });
            }
        });

    landmark_sub_ = sub;
    tf_filter_ = filter;
}

void LandmarkServerNode::create_polling_action_server() {
    std::string landmark_polling_action_name =
        this->declare_parameter<std::string>("action_servers.landmark_polling");
    landmark_polling_server_ =
        rclcpp_action::create_server<vortex_msgs::action::LandmarkPolling>(
            this, landmark_polling_action_name,

            [this](auto goal_id, auto goal) {
                return handle_landmark_polling_goal(goal_id, goal);
            },

            [this](auto goal_id) {
                return handle_landmark_polling_cancel(goal_id);
            },

            [this](auto goal_handle) {
                return handle_landmark_polling_accepted(goal_handle);
            });
}

void LandmarkServerNode::create_convergence_action_server() {
    std::string landmark_convergence_action_name =
        this->declare_parameter<std::string>(
            "action_servers.landmark_convergence");
    landmark_convergence_server_ =
        rclcpp_action::create_server<vortex_msgs::action::LandmarkConvergence>(
            this, landmark_convergence_action_name,

            [this](auto goal_id, auto goal) {
                return handle_landmark_convergence_goal(goal_id, goal);
            },

            [this](auto goal_id) {
                return handle_landmark_convergence_cancel(goal_id);
            },

            [this](auto goal_handle) {
                return handle_landmark_convergence_accepted(goal_handle);
            });
}

void LandmarkServerNode::create_reference_action_client() {
    std::string reference_action_name =
        this->declare_parameter<std::string>("action_servers.reference_filter");
    reference_filter_client_ = rclcpp_action::create_client<
        vortex_msgs::action::ReferenceFilterWaypoint>(this,
                                                      reference_action_name);
    if (!reference_filter_client_->wait_for_action_server(
            std::chrono::seconds(3))) {
        spdlog::warn("ReferenceFilter server not ready");
    }
}

std::vector<Landmark> LandmarkServerNode::ros_msg_to_landmarks(
    const vortex_msgs::msg::LandmarkArray& msg) const {
    std::vector<Landmark> out;
    out.reserve(msg.landmarks.size());
    for (const auto& lm_msg : msg.landmarks) {
        Landmark lm;
        lm.pose =
            vortex::utils::ros_conversions::ros_pose_to_pose(lm_msg.pose.pose);
        lm.class_key = vortex::filtering::LandmarkClassKey{
            lm_msg.type_class.type, lm_msg.type_class.subtype};
        out.push_back(lm);
    }
    return out;
}

RCLCPP_COMPONENTS_REGISTER_NODE(LandmarkServerNode)

}  // namespace vortex::mission
