#pragma once

#include <chrono>
#include <memory>
#include <optional>

#include <vortex/utils/types.hpp>

#include "dp_adapt_backs_controller_quat/dp_adapt_backs_controller.hpp"
#include "dp_adapt_backs_controller_quat/typedefs.hpp"

namespace vortex::control {

struct DPAdaptBacksControllerCoreConfig {
    DPAdaptParams controller_params;

    std::chrono::milliseconds time_step{10};

    bool initial_killswitch{true};

    vortex::utils::types::Mode initial_operation_mode{
        vortex::utils::types::Mode::manual};
};

class DPAdaptBacksControllerCore {
   public:
    explicit DPAdaptBacksControllerCore(
        const DPAdaptBacksControllerCoreConfig& config);

    void set_killswitch(bool killswitch_on);

    void set_operation_mode(vortex::utils::types::Mode mode);

    void set_pose(const vortex::utils::types::Pose& pose);

    void set_twist(const vortex::utils::types::Twist& twist);

    void set_guidance_pose(const vortex::utils::types::Pose& desired_pose);

    [[nodiscard]]
    Eigen::Vector6d tick(const vortex::utils::types::Pose& pose,
                         const vortex::utils::types::Pose& reference_pose,
                         const vortex::utils::types::Twist& twist);

    [[nodiscard]]
    bool killswitch_on() const noexcept;

    [[nodiscard]]
    vortex::utils::types::Mode operation_mode() const noexcept;

    void reset_controller_state();

   private:
    std::chrono::milliseconds time_step_{};

    vortex::utils::types::Pose pose_{};
    vortex::utils::types::Pose pose_d_{};
    vortex::utils::types::Twist twist_{};

    std::unique_ptr<DPAdaptBacksController> controller_{};

    bool killswitch_on_{true};

    vortex::utils::types::Mode operation_mode_{
        vortex::utils::types::Mode::manual};
};

}  // namespace vortex::control
