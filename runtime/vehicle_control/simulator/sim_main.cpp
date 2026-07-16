#include "config/nautilus_config.hpp"
#include "control/VehicleControl.hpp"
#include "simulator/StonefishIO.hpp"

int main(int argc, char** argv)
{
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    if (argc < 3) {
        std::cerr
            << "Usage: vehicle_control "
               "<stonefish_data_path> <scenario.scn>\n";

        return 1;
    }

    const std::string data_path = argv[1];
    const std::string scenario_path = argv[2];

    const auto config =
        vortex::runtime::vehicle_control::config::
            make_nautilus_config();

    VehicleControl vehicle_control{
        config.control,
    };

    StonefishIO simulator_io{
        config.stonefish,
        config.control.allocator.max_force,
    };

    vortex::simulation::stonefish::StonefishSimulator simulator{
        scenario_path,
        data_path,
        config.stonefish.update_frequency_hz,
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

            runtime_state.running = running;

            const SensorFrame sensors =
                simulator_io.read_sensors(manager);

            const ControlOutput output =
                vehicle_control.tick(
                    sensors,
                    runtime_state,
                    dt_s);

            simulator_io.apply_thrusters(
                manager,
                output);

            FrameMark;
        });

    return simulator.run();
}
