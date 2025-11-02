#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "vortex_msgs/action/reference_filter_waypoint.hpp"
#include <vortex/utils/math.hpp>

using namespace vortex::utils::math; // for converting euler to quaternion
using namespace std::chrono_literals;
using ReferenceFilterWaypoint = vortex_msgs::action::ReferenceFilterWaypoint;

static rclcpp::Node::SharedPtr g_ros_node = nullptr;

class SendWaypoint : public BT::SyncActionNode
{
public:
	SendWaypoint(const std::string& name, const BT::NodeConfiguration& config)
	: BT::SyncActionNode(name, config)
	{}

	static BT::PortsList providedPorts()
	{
		return {
			BT::InputPort<double>("x"),
			BT::InputPort<double>("y"),
			BT::InputPort<double>("z"),
			BT::InputPort<std::string>("frame_id"),
			BT::InputPort<std::string>("action_name")
		};
	}

	BT::NodeStatus tick() override
	{
		if (!g_ros_node) {
			RCLCPP_ERROR(rclcpp::get_logger("orca_simple_bt"), "Global ROS node not set");
			return BT::NodeStatus::FAILURE;
		}

		double x, y, z;
		std::string frame_id, action_name;
		if (!getInput("x", x) || !getInput("y", y) || !getInput("z", z) ||
				!getInput("frame_id", frame_id) || !getInput("action_name", action_name))
		{
			RCLCPP_ERROR(g_ros_node->get_logger(), "Missing input ports for SendWaypoint");
			return BT::NodeStatus::FAILURE;
		}

		auto client = rclcpp_action::create_client<ReferenceFilterWaypoint>(g_ros_node, action_name);

		if (!client->wait_for_action_server(1s)) {
			RCLCPP_WARN(g_ros_node->get_logger(), "Waiting for action server '%s'...", action_name.c_str());
			auto start = g_ros_node->now();
			while (!client->wait_for_action_server(200ms)) {
				rclcpp::spin_some(g_ros_node);
				if ((g_ros_node->now() - start) > rclcpp::Duration(5s)) {
					RCLCPP_ERROR(g_ros_node->get_logger(), "Action server '%s' not available", action_name.c_str());
					return BT::NodeStatus::FAILURE;
				}
			}
		}

		ReferenceFilterWaypoint::Goal goal_msg;
		goal_msg.goal.header.stamp = g_ros_node->now();
		goal_msg.goal.header.frame_id = frame_id;
		goal_msg.goal.pose.position.x = x;
		goal_msg.goal.pose.position.y = y;
		goal_msg.goal.pose.position.z = z;
		goal_msg.goal.pose.orientation.w = 1.0;
		goal_msg.goal.pose.orientation.x = 0.0;
		goal_msg.goal.pose.orientation.y = 0.0;
		goal_msg.goal.pose.orientation.z = euler_to_quat(0.0, 0.0, 1.57);;

		RCLCPP_INFO(g_ros_node->get_logger(),
								"BT: Sending waypoint -> [x=%.2f, y=%.2f, z=%.2f]", x, y, z);

		rclcpp_action::Client<ReferenceFilterWaypoint>::SendGoalOptions opts;
		opts.goal_response_callback = [g = g_ros_node](auto handle) {
			if (!handle) {
				RCLCPP_ERROR(g->get_logger(), "Goal rejected by Reference Filter server.");
			} else {
				RCLCPP_INFO(g->get_logger(), "Goal accepted by server.");
			}
		};
		opts.feedback_callback = [g = g_ros_node](auto, const std::shared_ptr<const ReferenceFilterWaypoint::Feedback>) {
			RCLCPP_INFO_THROTTLE(g->get_logger(), *g->get_clock(), 1000, "Feedback: Reference filter running...");
		};

		auto goal_handle_future = client->async_send_goal(goal_msg, opts);

		if (rclcpp::spin_until_future_complete(g_ros_node, goal_handle_future, 5s)
				!= rclcpp::FutureReturnCode::SUCCESS)
		{
			RCLCPP_ERROR(g_ros_node->get_logger(), "Failed to send goal (timeout).");
			return BT::NodeStatus::FAILURE;
		}

		auto goal_handle = goal_handle_future.get();
		if (!goal_handle) {
			RCLCPP_ERROR(g_ros_node->get_logger(), "Goal was rejected by server");
			return BT::NodeStatus::FAILURE;
		}

		auto result_future2 = client->async_get_result(goal_handle);
		auto start_wait = g_ros_node->now();
		while (rclcpp::spin_until_future_complete(g_ros_node, result_future2, 200ms) != rclcpp::FutureReturnCode::SUCCESS) {
			if ((g_ros_node->now() - start_wait) > rclcpp::Duration(120s)) {
				RCLCPP_ERROR(g_ros_node->get_logger(), "Result wait timeout");
				return BT::NodeStatus::FAILURE;
			}
		}

		auto wrapped_result = result_future2.get();
		if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
			RCLCPP_INFO(g_ros_node->get_logger(), "Waypoint reached successfully.");
			return BT::NodeStatus::SUCCESS;
		} else {
			RCLCPP_WARN(g_ros_node->get_logger(), "Waypoint failed (code %d)", static_cast<int>(wrapped_result.code));
			return BT::NodeStatus::FAILURE;
		}
	}
};

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	g_ros_node = std::make_shared<rclcpp::Node>("orca_simple_bt_node");

	std::vector<std::array<double,3>> waypoints = {
		{0.0, 0.0, 1.0},
		//{2.0, 2.0, 3.0},
		//{0.0, 2.0, 1.0},
		{0.0, 7.5, 1.0}
	};
	std::string frame_id = "base_link";
	std::string action_name = "/orca/reference_filter";

	BT::BehaviorTreeFactory factory;
	factory.registerNodeType<SendWaypoint>("SendWaypoint");

	std::ostringstream xml;
	xml << R"(<?xml version="1.0"?>)";
	xml << R"(<root main_tree_to_execute="MainTree">)";
	xml << R"(<BehaviorTree ID="MainTree">)";
	xml << R"(<Sequence name="MainSequence">)";
	for (size_t i = 0; i < waypoints.size(); ++i) {
		xml << "<SendWaypoint x=\"" << waypoints[i][0]
				<< "\" y=\"" << waypoints[i][1]
				<< "\" z=\"" << waypoints[i][2]
				<< "\" frame_id=\"" << frame_id
				<< "\" action_name=\"" << action_name
				<< "\" />";
	}
	xml << R"(</Sequence>)";
	xml << R"(</BehaviorTree>)";
	xml << R"(</root>)";

	auto tree = factory.createTreeFromText(xml.str());

	RCLCPP_INFO(g_ros_node->get_logger(), "Starting BT tree with %zu waypoints", waypoints.size());

	BT::NodeStatus status = BT::NodeStatus::RUNNING;
	while (status == BT::NodeStatus::RUNNING) {
		status = tree.tickRoot();
		rclcpp::spin_some(g_ros_node);
		std::this_thread::sleep_for(50ms);
	}

	if (status == BT::NodeStatus::SUCCESS) {
		RCLCPP_INFO(g_ros_node->get_logger(), "Mission complete! All waypoints reached.");
	} else {
		RCLCPP_WARN(g_ros_node->get_logger(), "BehaviorTree finished with failure.");
	}

	rclcpp::shutdown();
	return (status == BT::NodeStatus::SUCCESS) ? 0 : 1;
}
