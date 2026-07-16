#include "vehicle_control.hpp"
#include <tracy/Tracy.hpp>


namespace {


vortex::utils::types::Pose make_pose(const Eigen::Vector3d& position_world,
                                     const Eigen::Quaterniond& q_world_body) {
    return vortex::utils::types::Pose::from_eigen(position_world, q_world_body);
}


vortex::utils::types::Twist make_twist(
    const Eigen::Vector3d& velocity_body_m_s,
    const Eigen::Vector3d& angular_velocity_body_rad_s)
{
    vortex::utils::types::Twist twist{};

    twist.u = velocity_body_m_s.x();
    twist.v = velocity_body_m_s.y();
    twist.w = velocity_body_m_s.z();

    twist.p = angular_velocity_body_rad_s.x();
    twist.q = angular_velocity_body_rad_s.y();
    twist.r = angular_velocity_body_rad_s.z();

    return twist;
}

}  // namespace





namespace vortex::runtime::vehicle_control {

VehicleControl::VehicleControl(
    const config::VehicleConfig& config)
    : guidance_{config.guidance},
      controller_{config.controller},
      allocator_{config.allocator},
      eskf_{config.eskf},
      dvl_measurement_noise_{config.dvl_measurement_noise},
      pressure_measurement_noise_pa2_{
          config.pressure_measurement_noise_pa2}
{
}


ControlOutput VehicleControl::tick(
    const SensorFrame& sensors,
    const RuntimeState& runtime,
    double dt_s)
{
    ZoneScopedN("Vehicle control tick");

    if (!runtime.running) {
        ZoneScopedN("Stopped state");

        if (was_autonomous_enabled_) {
            reset_controller();
        }

        was_autonomous_enabled_ = false;
        return {};
    }

    update_estimator(sensors, dt_s);

    if (!eskf_initialized_) {
        ZoneScopedN("Waiting for ESKF initialization");
        return {};
    }

    const NominalState& nominal_state = eskf_.get_nominal_state();

    const Eigen::Vector3d velocity_body =
        nominal_state.quat.conjugate() * nominal_state.vel;

    const Eigen::Vector3d angular_velocity_body =
        latest_gyro_measurement_ - nominal_state.gyro_bias;

    /*
     * Prefer changing make_twist() so that it takes physical quantities
     * directly, rather than simulator-specific sensor structs.
     */
    const auto pose =
        make_pose(nominal_state.pos, nominal_state.quat);

    const auto twist =
        make_twist(velocity_body, angular_velocity_body);

    /*
     * This is depth, not altitude.
     *
     * Altitude normally means distance above the seabed, while pressure
     * gives distance below the water surface.
     */
    const double depth_m =
        sensors.depth ? sensors.depth->depth_m : nominal_state.pos.z();

    const auto reference = guidance_.tick(pose, depth_m);

    const bool autonomous_enabled =
        !runtime.killswitch_on &&
        runtime.operation_mode ==
            vortex::utils::types::Mode::autonomous;

    if (was_autonomous_enabled_ && !autonomous_enabled) {
        ZoneScopedN("Reset controller");
        reset_controller();
    }

    was_autonomous_enabled_ = autonomous_enabled;

    if (!autonomous_enabled || !reference.active) {
        return {};
    }

    Eigen::Vector6d commanded_wrench = Eigen::Vector6d::Zero();

    {
        ZoneScopedN("Controller");

        commanded_wrench =
            controller_.calculate_tau(
                pose,
                reference.pose,
                twist);
    }

    decltype(allocator_.allocate_thrust(commanded_wrench)) forces;

    {
        ZoneScopedN("Thrust allocation");
        forces = allocator_.allocate_thrust(commanded_wrench);
    }

    if (!forces) {
        ZoneScopedN("Allocation failure handling");

        reset_controller();
        return {};
    }

    return ControlOutput{
        .thrusters_enabled = true,
        .forces_n = *forces,
    };
}


void VehicleControl::update_estimator(
    const SensorFrame& sensors,
    double dt_s)
{
    ZoneScopedN("State estimation");

    if (sensors.imu && dt_s > 0.0) {
        ZoneScopedN("ESKF IMU update");

        latest_gyro_measurement_ =
            sensors.imu->angular_velocity_rad_s;

        const ImuMeasurement measurement{
            .accel = sensors.imu->linear_acceleration_m_s2,
            .gyro = sensors.imu->angular_velocity_rad_s,
        };

        eskf_.imu_update(measurement, dt_s);
        eskf_initialized_ = true;
    }

    if (eskf_initialized_ && sensors.dvl) {
        ZoneScopedN("ESKF DVL update");

        const SensorDVL measurement{
            .measurement = sensors.dvl->velocity_body_m_s,
            .measurement_noise = dvl_measurement_noise_,
        };

        eskf_.dvl_update(measurement);
    }

    if (eskf_initialized_ && sensors.depth) {
        ZoneScopedN("ESKF depth update");

        const SensorDepth measurement{
            .measurement = sensors.depth->depth_m,
            .measurement_noise = pressure_measurement_noise_pa2_,
        };

        eskf_.depth_update(measurement);
    }
}

void VehicleControl::reset_controller()
{
    controller_.reset_adap_param();
    controller_.reset_d_est();
}

} // namespace vortex::runtime::vehicle_control
