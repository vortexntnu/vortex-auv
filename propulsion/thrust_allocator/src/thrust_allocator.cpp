#include "vortex/propulsion/thrust_allocator/thrust_allocator.hpp"

#include <stdexcept>

#include "vortex/propulsion/thrust_allocator/allocator_factory.hpp"
#include "vortex/propulsion/thrust_allocator/thrust_allocator_utils.hpp"
#include "vortex/utils/math.hpp"

namespace vortex::propulsion {

ThrustAllocator::ThrustAllocator(
    const ThrustAllocatorSettings& settings)
    : num_thrusters_(
          static_cast<std::size_t>(
              settings.thruster_force_direction.cols()))
{
    if (settings.thruster_position.cols() !=
        settings.thruster_force_direction.cols()) {
        throw std::invalid_argument(
            "Thruster position and direction matrices must contain "
            "the same number of thrusters");
    }

    if (settings.input_weights.size() !=
        settings.thruster_force_direction.cols()) {
        throw std::invalid_argument(
            "input_weights size must equal number of thrusters");
    }

    const auto thrust_configuration =
        vortex::utils::math::build_thrust_configuration_matrix(
            settings.thruster_force_direction,
            settings.thruster_position,
            settings.center_of_mass);

    min_force_ =
        Eigen::VectorXd::Constant(num_thrusters_, settings.min_force);

    max_force_ =
        Eigen::VectorXd::Constant(num_thrusters_, settings.max_force);

    tau_max_ =
        vortex::utils::math::calculate_valid_thrust_region_polyhedron(
            thrust_configuration,
            min_force_,
            max_force_);

    AllocatorConfig config{
        .extended_thrust_matrix = thrust_configuration,
        .min_force = min_force_,
        .max_force = max_force_,
        .input_weight_matrix = settings.input_weights.asDiagonal(),
        .slack_weight_matrix = settings.slack_weights.asDiagonal(),
    };

    allocator_ = Factory::make_allocator(settings.solver_type, config);

    if (!allocator_) {
        throw std::runtime_error("Failed to create thrust allocator");
    }
}

std::optional<Eigen::VectorXd> ThrustAllocator::allocate_thrust(
    const BodyWrench& wrench) const
{
    if (!healthy_wrench(wrench)) {
        return std::nullopt;
    }

    const auto normalized_wrench =
        normalize_wrench_vector(wrench, tau_max_);

    auto forces = allocator_->calculate_allocated_thrust(
        normalized_wrench);

    if (!forces || is_invalid_matrix(*forces)) {
        return std::nullopt;
    }

    saturate_vector_values(*forces, min_force_, max_force_);

    return forces;
}

Eigen::VectorXd ThrustAllocator::zero_forces() const
{
    return Eigen::VectorXd::Zero(num_thrusters_);
}

std::size_t ThrustAllocator::num_thrusters() const noexcept
{
    return num_thrusters_;
}

bool ThrustAllocator::healthy_wrench(
    const BodyWrench& wrench) const
{
    if (is_invalid_matrix(wrench)) {
        return false;
    }

    return (wrench.cwiseAbs().cwiseQuotient(tau_max_)).maxCoeff() <= 1.0;
}

}  // namespace vortex::propulsion
