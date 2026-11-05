#ifndef FACTOR_GRAPH_LOCALIZATION__ROS__NAVIGATION_NODE_HPP_
#define FACTOR_GRAPH_LOCALIZATION__ROS__NAVIGATION_NODE_HPP_

#include <rclcpp/node.hpp>

#include <array>

#include "factor_graph_localization/factor_builders/dvl_beam.hpp"

namespace vortex::navigation {

/** Localization node with fixed STIM300/Nucleus calibration loaded at startup.
 */
class NavigationNode : public rclcpp::Node {
   public:
    explicit NavigationNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    /// Calibration for Nucleus beams 1, 2, and 3, respectively.
    const std::array<DvlBeamGeometry, 3>& dvl_beam_geometries() const {
        return dvl_beam_geometries_;
    }

   private:
    std::array<DvlBeamGeometry, 3> dvl_beam_geometries_;
};

}  // namespace vortex::navigation

#endif  // FACTOR_GRAPH_LOCALIZATION__ROS__NAVIGATION_NODE_HPP_
