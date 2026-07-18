#include "common/control_output.hpp"
#include "common/manual_command.hpp"
#include "common/runtime_state.hpp"
#include "common/sensor_frame.hpp"
#include "config/nautilus_config.hpp"
#include "control/vehicle_control.hpp"
#include "simulator/keyboard_controller.hpp"
#include "simulator/stonefish_io.hpp"

#include <tracy/Tracy.hpp>
#include <vortex/simulator/stonefish/simulator.hpp>

#include <boost/asio.hpp>

#include <vortex/telemetry/telemetry_publisher.hpp>

#include <thread>

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>

namespace {

std::atomic_bool running{true};

void handle_signal(int) {
    running.store(false);
}
[[nodiscard]]
std::uint64_t to_timestamp_ns(
    vortex::runtime::vehicle_control::SteadyTimePoint timestamp) {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            timestamp.time_since_epoch())
            .count());
}

}  // namespace

int main(int argc, char** argv) {
    using namespace vortex::runtime::vehicle_control;

    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    if (argc < 3) {
        std::cerr << "Usage: vehicle_control_simulator "
                     "<stonefish_data_path> <scenario.scn>\n";

        return 1;
    }

    boost::asio::io_context telemetry_io_context;

    const auto telemetry_work_guard =
        boost::asio::make_work_guard(telemetry_io_context);

    vortex::telemetry::telemetry_publisher telemetry{telemetry_io_context,
                                                     "127.0.0.1", 9870};

    std::jthread telemetry_thread{
        [&telemetry_io_context]() { telemetry_io_context.run(); }};

    const std::string data_path = argv[1];
    const std::string scenario_path = argv[2];

    constexpr double simulator_frequency_hz = 500.0;

    constexpr auto control_period = std::chrono::milliseconds{2};

    const config::VehicleConfig vehicle_config =
        config::make_nautilus_config(control_period);

    VehicleControl vehicle_control{
        vehicle_config,
    };

    StonefishIo stonefish_io{
        StonefishIoConfig{
            .imu_sensor_name = "nautilus/imu_link",

            .pressure_sensor_name = "nautilus/pressure_sensor_link",

            .dvl_sensor_name = "nautilus/dvl_link",

            .max_thruster_force_n = vehicle_config.allocator.max_force,
        },
    };

    vortex::simulation::stonefish::StonefishSimulator simulator{
        scenario_path,
        data_path,
        simulator_frequency_hz,
    };

    KeyboardController keyboard_controller;
    keyboard_controller.start();

    RuntimeState runtime_state{
        .running = true,
        .killswitch_on = false,
        .operation_mode = vortex::utils::types::Mode::manual,
    };

    simulator.set_step_callback(
        [&](vortex::simulation::stonefish::VortexSimulationManager& manager,
            double dt_s) {
            ZoneScopedN("Simulation step callback");

            runtime_state.running = running.load();

            if (!runtime_state.running) {
                stonefish_io.stop_thrusters(manager);
                return;
            }

            const SensorFrame sensors = stonefish_io.read_sensors(manager);

            const ManualCommand manual_command = keyboard_controller.command();

            const ControlOutput output = vehicle_control.tick(
                sensors, runtime_state, manual_command, dt_s);

            stonefish_io.apply_thrusters(manager, output);

            const auto timestamp_ns =
                vortex::telemetry::telemetry_publisher::monotonic_time_ns();

            if (sensors.imu.has_value()) {
                const ImuSample& sample = *sensors.imu;

                const vortex::telemetry::imu_data message{
                    .angular_velocity_rad_s =
                        {
                            .x = sample.angular_velocity_rad_s.x(),
                            .y = sample.angular_velocity_rad_s.y(),
                            .z = sample.angular_velocity_rad_s.z(),
                        },
                    .linear_acceleration_m_s2 =
                        {
                            .x = sample.linear_acceleration_m_s2.x(),
                            .y = sample.linear_acceleration_m_s2.y(),
                            .z = sample.linear_acceleration_m_s2.z(),
                        },
                };

                telemetry.publish(to_timestamp_ns(sample.timestamp), message);
            }

            if (sensors.dvl.has_value()) {
                const DvlSample& sample = *sensors.dvl;

                const vortex::telemetry::dvl_data message{
                    .velocity_m_s =
                        {
                            .x = sample.velocity_m_s.x(),
                            .y = sample.velocity_m_s.y(),
                            .z = sample.velocity_m_s.z(),
                        },
                    .altitude_m = sample.altitude_m,
                    .velocity_quality = sample.velocity_quality,
                    .velocity_valid = sample.velocity_valid,
                    .altitude_valid = sample.altitude_valid,
                };

                telemetry.publish(to_timestamp_ns(sample.timestamp), message);
            }

            if (sensors.depth.has_value()) {
                const DepthSample& sample = *sensors.depth;

                const vortex::telemetry::pressure_data message{
                    .pressure_pa = sample.pressure_pa,
                    .temperature_c = sample.temperature_c,
                    .depth_m = sample.depth_m,
                };

                telemetry.publish(to_timestamp_ns(sample.timestamp), message);
            }

            FrameMark;
        });

    simulator.run_graphical();

    return 0;
}
