#include "config/nautilus_config.hpp"
#include "control/VehicleControl.hpp"
#include "hardware/HardwareIO.hpp"

int main()
{
    const auto config =
        vortex::runtime::vehicle_control::config::
            make_nautilus_config();

    VehicleControl vehicle_control{
        config.control,
    };

    HardwareIO hardware_io{
        config.hardware,
    };

    RuntimeState runtime_state{};

    auto previous_time =
        std::chrono::steady_clock::now();

    while (running) {
        const auto current_time =
            std::chrono::steady_clock::now();

        const double dt_s =
            std::chrono::duration<double>(
                current_time - previous_time)
                .count();

        previous_time = current_time;

        runtime_state.running = running;
        runtime_state.killswitch_on =
            hardware_io.killswitch_on();

        runtime_state.operation_mode =
            hardware_io.operation_mode();

        const SensorFrame sensors =
            hardware_io.read_sensors();

        const ControlOutput output =
            vehicle_control.tick(
                sensors,
                runtime_state,
                dt_s);

        hardware_io.apply_thrusters(output);
    }

    hardware_io.disable_thrusters();

    return 0;
}
