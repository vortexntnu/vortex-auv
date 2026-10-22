#include "factor_graph_localization/ros/navigation_node.hpp"

#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include <stdexcept>

#include "factor_graph_localization/configuration/dvl_calibration.hpp"

namespace vortex::navigation {
namespace {

std::array<DvlBeamGeometry, 3> load_calibration(rclcpp::Node& node) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description =
        "Path to the STIM300/Nucleus calibration YAML, loaded at startup.";
    descriptor.read_only = true;
    const auto config_path =
        node.declare_parameter<std::string>("sensors_config_file", descriptor);
    if (config_path.empty()) {
        throw std::invalid_argument("sensors_config_file must be a file path");
    }
    return load_dvl_beam_geometries(config_path);
}

}  // namespace

NavigationNode::NavigationNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("factor_graph_localization", options),
      dvl_beam_geometries_(load_calibration(*this)) {}

}  // namespace vortex::navigation
