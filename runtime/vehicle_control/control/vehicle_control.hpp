#pragma once

#include "common/control_output.hpp"
#include "common/manual_command.hpp"
#include "common/runtime_state.hpp"
#include "common/sensor_frame.hpp"
#include "config/vehicle_config.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace vortex::runtime::vehicle_control {

class VehicleControl {
   public:
    explicit VehicleControl(const config::VehicleConfig& config);

    ControlOutput tick(const SensorFrame& sensors,
                       const RuntimeState& runtime,
                       const ManualCommand& manual_command,
                       double dt_s);

   private:
    void update_estimator(const SensorFrame& sensors, double dt_s);

    void reset_controller();

    vortex::guidance::WaypointGuidanceManager guidance_;
    vortex::control::DPAdaptBacksController controller_;
    vortex::propulsion::ThrustAllocator allocator_;
    ESKF eskf_;

    Eigen::Matrix3d dvl_measurement_noise_;
    double pressure_measurement_noise_pa2_ = 0.0;

    bool was_autonomous_enabled_ = false;
    bool eskf_initialized_ = false;

    Eigen::Vector3d latest_gyro_measurement_ = Eigen::Vector3d::Zero();
};

}  // namespace vortex::runtime::vehicle_control
