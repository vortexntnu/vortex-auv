#include "control/vehicle_control.hpp"

#include <tracy/Tracy.hpp>

namespace {

vortex::utils::types::Pose make_pose(
    const Eigen::Vector3d& position_world,
    const Eigen::Quaterniond& q_world_body)
{
    vortex::utils::types::Pose pose{};

    pose.x = position_world.x();
    pose.y = position_world.y();
    pose.z = position_world.z();

    pose.qw = q_world_body.w();
    pose.qx = q_world_body.x();
    pose.qy = q_world_body.y();
    pose.qz = q_world_body.z();

    return pose;
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
    const ManualCommand& manual_command,
    double dt_s)
{
    ZoneScopedN("Vehicle control tick");

    if (!runtime.running || runtime.killswitch_on) {
        if (was_autonomous_enabled_) {
            reset_controller();
        }

        was_autonomous_enabled_ = false;
        return {};
    }

    update_estimator(sensors, dt_s);

    const bool manual_enabled =
        runtime.operation_mode ==
            vortex::utils::types::Mode::manual &&
        manual_command.active;

    if (manual_enabled) {
        const auto forces =
            allocator_.allocate_thrust(
                manual_command.wrench);

        if (!forces) {
            reset_controller();
            return {};
        }

        was_autonomous_enabled_ = false;

        return ControlOutput{
            .thrusters_enabled = true,
            .forces_n = *forces,
        };
    }

    if (!eskf_initialized_) {
        return {};
    }

    const NominalState& nominal_state =
        eskf_.get_nominal_state();

    const Eigen::Vector3d estimated_velocity_body =
        nominal_state.quat.conjugate() *
        nominal_state.vel;

    const Eigen::Vector3d estimated_angular_velocity_body =
        latest_gyro_measurement_ -
        nominal_state.gyro_bias;

    const auto pose =
        make_pose(
            nominal_state.pos,
            nominal_state.quat);

    const auto twist =
        make_twist(
            estimated_velocity_body,
            estimated_angular_velocity_body);

    const double depth_m =
        sensors.depth
            ? sensors.depth->depth_m
            : nominal_state.pos.z();

    const auto reference =
        guidance_.tick(pose, depth_m);

    const bool autonomous_enabled =
        runtime.operation_mode ==
            vortex::utils::types::Mode::autonomous;

    if (was_autonomous_enabled_ &&
        !autonomous_enabled) {
        reset_controller();
    }

    was_autonomous_enabled_ =
        autonomous_enabled;

    Eigen::Matrix<double, 6, 1> commanded_wrench =
        Eigen::Matrix<double, 6, 1>::Zero();

    if (autonomous_enabled &&
        reference.active) {
        commanded_wrench =
            controller_.calculate_tau(
                pose,
                reference.pose,
                twist);
    }

    const auto forces =
        allocator_.allocate_thrust(
            commanded_wrench);

    if (!forces) {
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
    if (sensors.imu && dt_s > 0.0) {
        latest_gyro_measurement_ =
            sensors.imu->angular_velocity_rad_s;

        const ImuMeasurement imu_measurement{
            .accel =
                sensors.imu->linear_acceleration_m_s2,
            .gyro =
                sensors.imu->angular_velocity_rad_s,
        };

        eskf_.imu_update(
            imu_measurement,
            dt_s);

        eskf_initialized_ = true;
    }

    if (eskf_initialized_ && sensors.dvl) {
        const SensorDVL dvl_measurement{
            .measurement =
                sensors.dvl->velocity_body_m_s,
            .measurement_noise =
                dvl_measurement_noise_,
        };

        eskf_.dvl_update(
            dvl_measurement);
    }

    if (eskf_initialized_ && sensors.depth) {
        const SensorDepth depth_measurement{
            .measurement =
                sensors.depth->depth_m,
            .measurement_noise =
                pressure_measurement_noise_pa2_,
        };

        eskf_.depth_update(
            depth_measurement);
    }
}

void VehicleControl::reset_controller()
{
    controller_.reset_adap_param();
    controller_.reset_d_est();
}

}  // namespace vortex::runtime::vehicle_control
