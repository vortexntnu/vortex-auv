#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/component_manager.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto manager = std::make_shared<rclcpp_components::ComponentManager>(rclcpp::NodeOptions());
    auto node_options = rclcpp::NodeOptions().use_intra_process_comms(true);

    manager->load_component(
        "dp_adapt_backs_controller_component",
        "dp_adapt_backs_controller::DPAdaptBacksControllerComponent",
        {},
        node_options);
    
    rclcpp::spin(manager);
    rclcpp::shutdown();
}