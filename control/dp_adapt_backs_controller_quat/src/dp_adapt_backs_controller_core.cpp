#include "dp_adapt_backs_controller_quat/dp_adapt_backs_controller_core.hpp"

#include <stdexcept>

namespace vortex::control {

DPAdaptBacksControllerCore::DPAdaptBacksControllerCore(
    const DPAdaptBacksControllerCoreConfig& config)
    : time_step_(config.time_step),
      controller_(std::make_unique<DPAdaptBacksController>(
          config.controller_params)),
      killswitch_on_(config.initial_killswitch),
      operation_mode_(config.initial_operation_mode)
{
    if (!controller_) {
        throw std::runtime_error("Failed to create DP adaptive controller");
    }
}

void DPAdaptBacksControllerCore::set_killswitch(bool killswitch_on)
{
    killswitch_on_ = killswitch_on;

    if (controller_) {
        controller_->reset_adap_param();
        controller_->reset_d_est();
    }
}

void DPAdaptBacksControllerCore::set_operation_mode(
    vortex::utils::types::Mode mode)
{
    operation_mode_ = mode;

    if (operation_mode_ == vortex::utils::types::Mode::autonomous ||
        operation_mode_ == vortex::utils::types::Mode::reference) {
        pose_d_ = pose_;
    }
}

void DPAdaptBacksControllerCore::set_pose(
    const vortex::utils::types::Pose& pose)
{
    pose_ = pose;
}

void DPAdaptBacksControllerCore::set_twist(
    const vortex::utils::types::Twist& twist)
{
    twist_ = twist;
}

void DPAdaptBacksControllerCore::set_guidance_pose(
    const vortex::utils::types::Pose& desired_pose)
{
    pose_d_ = desired_pose;
}

std::optional<Eigen::Vector6d> DPAdaptBacksControllerCore::tick()
{
    if (killswitch_on_ ||
        operation_mode_ == vortex::utils::types::Mode::manual) {
        return std::nullopt;
    }

    return controller_->calculate_tau(pose_, pose_d_, twist_);
}

bool DPAdaptBacksControllerCore::killswitch_on() const noexcept
{
    return killswitch_on_;
}

vortex::utils::types::Mode
DPAdaptBacksControllerCore::operation_mode() const noexcept
{
    return operation_mode_;
}

void DPAdaptBacksControllerCore::reset_controller_state()
{
    if (!controller_) {
        return;
    }

    controller_->reset_adap_param();
    controller_->reset_d_est();
}

}  // namespace vortex::control
