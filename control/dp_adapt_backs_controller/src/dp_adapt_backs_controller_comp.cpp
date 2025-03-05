#include "rclcpp/rclcpp.hpp"
#include "dp_adapt_backs_controller/dp_adapt_backs_controller_ros.hpp"
#include "rclcpp_components/register_node_macro.hpp"

class DPAdaptBacksControllerComponent : public DPAdaptBacksControllerNode {
    public:
        explicit DPAdaptBacksControllerComponent(const rclcpp::NodeOptions & options)
        : DPAdaptBacksControllerNode(options) {}
};

RCLCPP_COMPONENTS_REGISTER_NODE(DPAdaptBacksControllerComponent)