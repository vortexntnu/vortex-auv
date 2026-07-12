#include <algorithm>
#include <array>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <string>

#include <tracy/Tracy.hpp>

#include <vortex/propulsion/thrust_allocator/thrust_allocator.hpp>
#include <vortex/simulator/stonefish/simulator.hpp>

#include "dp_adapt_backs_controller_quat/dp_adapt_backs_controller.hpp"
#include "reference_filter_dp_quat/waypoint_guidance_manager.hpp"

#include "eskf/eskf.hpp"

namespace {

volatile std::sig_atomic_t running = 1;

void handle_signal(int) {
    running = 0;
}
constexpr auto control_period = std::chrono::milliseconds{10};

Eigen::Quaterniond quat_from_xyzw(const std::array<double, 4>& xyzw) {
    Eigen::Quaterniond q{
        xyzw[3],
        xyzw[0],
        xyzw[1],
        xyzw[2],
    };

    if (q.norm() < 1e-9) {
        return Eigen::Quaterniond::Identity();
    }

    return q.normalized();
}

vortex::utils::types::Pose make_pose(const Eigen::Vector3d& position_world,
                                     const Eigen::Quaterniond& q_world_body) {
    return vortex::utils::types::Pose::from_eigen(position_world, q_world_body);
}

vortex::utils::types::Twist make_twist(
    const vortex::simulation::stonefish::DvlReading& dvl,
    const vortex::simulation::stonefish::ImuReading& imu) {
    vortex::utils::types::Twist twist{};

    if (dvl.valid) {
        twist.u = dvl.velocity_body_m_s[0];
        twist.v = dvl.velocity_body_m_s[1];
        twist.w = dvl.velocity_body_m_s[2];
    }

    if (imu.valid) {
        twist.p = imu.angular_velocity_rad_s[0];
        twist.q = imu.angular_velocity_rad_s[1];
        twist.r = imu.angular_velocity_rad_s[2];
    }

    return twist;
}

vortex::simulation::stonefish::ThrusterCommand make_sim_thruster_command(
    const Eigen::VectorXd& forces,
    double max_force) {
    vortex::simulation::stonefish::ThrusterCommand command{};

    const auto n = std::min<Eigen::Index>(
        static_cast<Eigen::Index>(command.command.size()), forces.size());

    for (Eigen::Index i = 0; i < n; ++i) {
        const double normalized = forces(i) / max_force;

        command.command[static_cast<std::size_t>(i)] =
            std::clamp(normalized, -1.0, 1.0);
    }

    return command;
}

}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    if (argc < 3) {
        std::cerr
            << "Usage: vehicle_control <stonefish_data_path> <scenario.scn>\n";
        return 1;
    }

    const std::string data_path = argv[1];
    const std::string scenario_path = argv[2];

    vortex::guidance::WaypointGuidanceManagerConfig guidance_config{
        .filter_params =
            {
                .omega = Eigen::Vector6d::Constant(1.0),
                .zeta = Eigen::Vector6d::Constant(1.0),
            },
        .time_step = control_period,
        .altitude_control_enabled = false,
        .altitude_low_pass_alpha = 0.9,
    };

    vortex::control::DPAdaptParams controller_params{
        .adapt_param = Eigen::Vector12d::Zero(),
        .d_gain = Eigen::Vector6d::Ones(),
        .K1 = Eigen::Vector6d::Ones(),
        .K2 = Eigen::Vector6d::Ones(),

        .r_b_bg = Eigen::Vector3d::Zero(),
        .inertia_matrix_body = Eigen::Vector3d::Ones(),

        .mass_intertia_matrix = Eigen::Matrix6d::Identity(),

        .tau_max = Eigen::Vector6d::Constant(100.0),

        .mass = 10.0,

        .time_step_s = static_cast<double>(control_period.count()) / 1000.0,

        .singularity_tolerance = 1e-6,
        .adapt_param_max = 100.0,
        .d_est_max = 100.0,
    };

    constexpr Eigen::Index num_thrusters = 8;

    Eigen::MatrixXd dummy_thruster_directions(3, num_thrusters);
    dummy_thruster_directions << 1.0, 1.0, -1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        -1.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0,
        1.0;

    Eigen::MatrixXd dummy_thruster_positions(3, num_thrusters);
    dummy_thruster_positions << 0.30, 0.30, -0.30, -0.30, 0.25, 0.25, -0.25,
        -0.25, 0.20, -0.20, 0.20, -0.20, 0.20, -0.20, 0.20, -0.20, 0.00, 0.00,
        0.00, 0.00, 0.00, 0.00, 0.00, 0.00;

    vortex::propulsion::ThrustAllocatorSettings allocator_settings{
        .solver_type = "pseudoinverse",

        .center_of_mass = Eigen::Vector3d::Zero(),

        .thruster_force_direction = dummy_thruster_directions,
        .thruster_position = dummy_thruster_positions,

        .input_weights = Eigen::VectorXd::Ones(num_thrusters),
        .slack_weights = Eigen::VectorXd::Ones(6),

        .min_force = -50.0,
        .max_force = 50.0,
    };

    vortex::guidance::WaypointGuidanceManager guidance{guidance_config};
    vortex::control::DPAdaptBacksController controller{controller_params};
    vortex::propulsion::ThrustAllocator allocator{allocator_settings};

    bool killswitch_on = false;
    auto operation_mode = vortex::utils::types::Mode::autonomous;
    bool was_autonomous_enabled = false;

    Eigen::Vector3d position_world = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q_world_body = Eigen::Quaterniond::Identity();

    vortex::simulation::stonefish::StonefishSimulator sim{
        scenario_path,
        data_path,
        500.0,
    };

    EskfParams eskf_params{};

    eskf_params.Q = Eigen::Matrix12d::Identity() * 1e-3;

    eskf_params.P = Eigen::Matrix15d::Identity() * 1e-2;

    eskf_params.g_ = Eigen::Vector3d{0.0, 0.0, 9.82841};

    ESKF eskf{eskf_params};

    const Eigen::Matrix3d dvl_measurement_noise =
        Eigen::Vector3d{
            0.02 * 0.02,
            0.02 * 0.02,
            0.03 * 0.03,
        }
            .asDiagonal();

    constexpr double depth_std_m = 0.05;
    constexpr double depth_measurement_noise = depth_std_m * depth_std_m;

    bool eskf_initialized = false;

    Eigen::Vector3d latest_gyro_measurement = Eigen::Vector3d::Zero();

    sim.set_step_callback(
        [&](vortex::simulation::stonefish::VortexSimulationManager& sim_manager,
            double dt_s) {
            ZoneScopedN("Simulation step callback");

            if (!running) {
                ZoneScopedN("Stopped state");

                sim_manager.set_thrusters(
                    vortex::simulation::stonefish::ThrusterCommand{});

                return;
            }

            vortex::simulation::stonefish::ImuReading imu;
            vortex::simulation::stonefish::PressureReading pressure;
            vortex::simulation::stonefish::DvlReading dvl;

            {
                ZoneScopedN("Read sensors");

                {
                    ZoneScopedN("Read IMU");
                    imu = sim_manager.read_imu("nautilus/imu_link");
                }

                {
                    ZoneScopedN("Read pressure");
                    pressure = sim_manager.read_pressure(
                        "nautilus/pressure_sensor_link");
                }

                {
                    ZoneScopedN("Read DVL");
                    dvl = sim_manager.read_dvl("nautilus/dvl_link");
                }
            }
            {
                ZoneScopedN("State estimation");

                /*
                 * IMU prediction.
                 */
                if (imu.valid && dt_s > 0.0) {
                    ZoneScopedN("ESKF IMU update");

                    const Eigen::Vector3d measured_acceleration{
                        imu.linear_acceleration_m_s2[0],
                        imu.linear_acceleration_m_s2[1],
                        imu.linear_acceleration_m_s2[2],
                    };

                    latest_gyro_measurement = Eigen::Vector3d{
                        imu.angular_velocity_rad_s[0],
                        imu.angular_velocity_rad_s[1],
                        imu.angular_velocity_rad_s[2],
                    };

                    const ImuMeasurement imu_measurement{
                        .accel = measured_acceleration,
                        .gyro = latest_gyro_measurement,
                    };

                    eskf.imu_update(imu_measurement, dt_s);
                    eskf_initialized = true;
                }

                /*
                 * DVL correction.
                 *
                 * The simulator reports body-frame velocity, which is what
                 * SensorDVL appears intended to represent.
                 */
                if (eskf_initialized && dvl.valid) {
                    ZoneScopedN("ESKF DVL update");

                    const SensorDVL dvl_measurement{
                        .measurement =
                            Eigen::Vector3d{
                                dvl.velocity_body_m_s[0],
                                dvl.velocity_body_m_s[1],
                                dvl.velocity_body_m_s[2],
                            },
                        .measurement_noise = dvl_measurement_noise,
                    };

                    eskf.dvl_update(dvl_measurement);
                }

                /*
                 * Depth correction.
                 *
                 * This assumes the ESKF uses positive-down z, which is
                 * suggested by gravity being {0, 0, +9.82841}.
                 */
                if (eskf_initialized && pressure.valid) {
                    ZoneScopedN("ESKF depth update");

                    const SensorDepth depth_measurement{
                        .measurement = pressure.depth_m,
                        .measurement_noise = depth_measurement_noise,
                    };

                    eskf.depth_update(depth_measurement);
                }
            }

            /*
             * Do not run the controller before the ESKF has received its first
             * prediction step.
             */
            if (!eskf_initialized) {
                ZoneScopedN("Waiting for ESKF initialization");

                sim_manager.set_thrusters(
                    vortex::simulation::stonefish::ThrusterCommand{});

                FrameMark;
                return;
            }

            /*
             * Read the corrected nominal state after all measurement updates.
             */
            const NominalState& nominal_state = eskf.get_nominal_state();

            /*
             * Preserve these variables in case they are used elsewhere.
             */
            position_world = nominal_state.pos;
            q_world_body = nominal_state.quat;

            /*
             * The ESKF velocity is assumed to be in the world frame.
             * make_twist() currently expects the simulator DVL reading, which
             * contains body-frame velocity, so convert it back to the body
             * frame.
             */
            const Eigen::Vector3d estimated_velocity_body =
                nominal_state.quat.conjugate() * nominal_state.vel;

            /*
             * Remove the estimated gyroscope bias from the latest IMU
             * measurement.
             */
            const Eigen::Vector3d estimated_angular_velocity_body =
                latest_gyro_measurement - nominal_state.gyro_bias;

            /*
             * Create temporary simulator-style readings so the existing
             * make_twist(dvl, imu) function can remain unchanged.
             */
            auto estimated_dvl = dvl;

            estimated_dvl.velocity_body_m_s = {
                estimated_velocity_body.x(),
                estimated_velocity_body.y(),
                estimated_velocity_body.z(),
            };

            estimated_dvl.valid = true;

            auto estimated_imu = imu;

            estimated_imu.angular_velocity_rad_s = {
                estimated_angular_velocity_body.x(),
                estimated_angular_velocity_body.y(),
                estimated_angular_velocity_body.z(),
            };

            estimated_imu.orientation_xyzw = {
                nominal_state.quat.x(),
                nominal_state.quat.y(),
                nominal_state.quat.z(),
                nominal_state.quat.w(),
            };

            estimated_imu.valid = true;

            decltype(make_pose(position_world, q_world_body)) pose;
            decltype(make_twist(estimated_dvl, estimated_imu)) twist;

            {
                ZoneScopedN("Construct state from ESKF");

                pose = make_pose(nominal_state.pos, nominal_state.quat);

                twist = make_twist(estimated_dvl, estimated_imu);
            }

            const double altitude_m = pressure.valid ? pressure.depth_m : 0.0;

            decltype(guidance.tick(pose, altitude_m)) reference;

            {
                ZoneScopedN("Guidance");

                reference = guidance.tick(pose, altitude_m);
            }

            const bool autonomous_enabled =
                !killswitch_on &&
                operation_mode == vortex::utils::types::Mode::autonomous;

            Eigen::Vector6d commanded_wrench = Eigen::Vector6d::Zero();

            if (autonomous_enabled && reference.active) {
                ZoneScopedN("Controller");

                commanded_wrench =
                    controller.calculate_tau(pose, reference.pose, twist);
            }

            if (was_autonomous_enabled && !autonomous_enabled) {
                ZoneScopedN("Reset controller");

                controller.reset_adap_param();
                controller.reset_d_est();
            }

            was_autonomous_enabled = autonomous_enabled;

            decltype(allocator.allocate_thrust(commanded_wrench)) forces;

            {
                ZoneScopedN("Thrust allocation");

                forces = allocator.allocate_thrust(commanded_wrench);
            }

            if (!forces) {
                ZoneScopedN("Allocation failure handling");

                controller.reset_adap_param();
                controller.reset_d_est();

                sim_manager.set_thrusters(
                    vortex::simulation::stonefish::ThrusterCommand{});

                return;
            }

            decltype(make_sim_thruster_command(
                *forces, allocator_settings.max_force)) sim_thruster_command;

            {
                ZoneScopedN("Build thruster command");

                sim_thruster_command = make_sim_thruster_command(
                    *forces, allocator_settings.max_force);
            }

            {
                ZoneScopedN("Apply thrusters");

                sim_manager.set_thrusters(sim_thruster_command);
            }

            FrameMark;
        });

    sim.run_graphical();

    return 0;
}
