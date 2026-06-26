#pragma once

#include <Eigen/Dense>

#include <memory>
#include <optional>
#include <string>

#include "vortex/propulsion/thrust_allocator/allocator.hpp"

namespace vortex::propulsion {

using BodyWrench = Eigen::Matrix<double, 6, 1>;

struct ThrustAllocatorSettings {
    std::string solver_type;

    Eigen::Vector3d center_of_mass;

    Eigen::MatrixXd thruster_force_direction;
    Eigen::MatrixXd thruster_position;

    Eigen::VectorXd input_weights;
    Eigen::VectorXd slack_weights;

    double min_force;
    double max_force;
};

class ThrustAllocator {
public:
    explicit ThrustAllocator(const ThrustAllocatorSettings& settings);

    [[nodiscard]]
    std::optional<Eigen::VectorXd> allocate_thrust(
        const BodyWrench& wrench) const;

    [[nodiscard]]
    Eigen::VectorXd zero_forces() const;

    [[nodiscard]]
    std::size_t num_thrusters() const noexcept;

private:
    [[nodiscard]]
    bool healthy_wrench(const BodyWrench& wrench) const;

    std::unique_ptr<Allocator> allocator_;

    Eigen::VectorXd tau_max_;
    double min_force_;
    double max_force_;
    std::size_t num_thrusters_{};
};

}  // namespace vortex::propulsion
