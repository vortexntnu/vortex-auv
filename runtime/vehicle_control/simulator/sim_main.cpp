#include "common/control_output.hpp"
#include "common/runtime_state.hpp"
#include "common/sensor_frame.hpp"
#include "config/nautilus_config.hpp"
#include "control/vehicle_control.hpp"
#include "simulator/stonefish_io.hpp"

#include <vortex/simulator/stonefish/simulator.hpp>
#include <tracy/Tracy.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>

namespace {

std::atomic_bool running{true};

void handle_signal(int)
{
    running.store(false);
}

}  // namespace

int main(int argc, char** argv)
{
    using namespace vortex::runtime::vehicle_control;

    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    if (argc < 3) {
        std::cerr
            << "Usage: vehicle_control_simulator "
               "<stonefish_data_path> <scenario.scn>\n";

        return 1;
    }

    const std::string data_path = argv[1];
    const std::string scenario_path = argv[2];

    constexpr double simulator_frequency_hz = 500.0;

    constexpr auto control_period =
        std::chrono::milliseconds{2};

    const config::VehicleConfig vehicle_config =
        config::make_nautilus_config(control_period);

    VehicleControl vehicle_control{
        vehicle_config,
    };

    StonefishIo stonefish_io{
        StonefishIoConfig{
            .imu_sensor_name =
                "nautilus/imu_link",

            .pressure_sensor_name =
                "nautilus/pressure_sensor_link",

            .dvl_sensor_name =
                "nautilus/dvl_link",

            .max_thruster_force_n =
                vehicle_config.allocator.max_force,
        },
    };

    vortex::simulation::stonefish::StonefishSimulator simulator{
        scenario_path,
        data_path,
        simulator_frequency_hz,
    };

    RuntimeState runtime_state{
        .running = true,
        .killswitch_on = false,
        .operation_mode =
            vortex::utils::types::Mode::autonomous,
    };

    simulator.set_step_callback(
        [&](vortex::simulation::stonefish::
                VortexSimulationManager& manager,
            double dt_s)
        {
            ZoneScopedN("Simulation step callback");

            runtime_state.running =
                running.load();

            if (!runtime_state.running) {
                stonefish_io.stop_thrusters(manager);
                return;
            }

            const SensorFrame sensors =
                stonefish_io.read_sensors(manager);

            const ControlOutput output =
                vehicle_control.tick(
                    sensors,
                    runtime_state,
                    dt_s);

            stonefish_io.apply_thrusters(
                manager,
                output);

            FrameMark;
        });


    simulator.run_graphical();

    return 0;
}
