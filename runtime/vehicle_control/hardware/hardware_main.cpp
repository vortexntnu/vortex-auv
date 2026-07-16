
#include "common/control_output.hpp"
#include "common/runtime_state.hpp"
#include "common/sensor_frame.hpp"
#include "config/nautilus_config.hpp"
#include "control/vehicle_control.hpp"
#include "hardware/hardware_io.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <thread>

namespace {

std::atomic_bool running{true};

void handle_signal(int)
{
    running.store(false);
}

}  // namespace

int main()
{
    using namespace vortex::runtime::vehicle_control;

    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    constexpr auto control_period =
        std::chrono::milliseconds{10};

    const config::VehicleConfig vehicle_config =
        config::make_nautilus_config(control_period);

    VehicleControl vehicle_control{
        vehicle_config,
    };

    /*
     * Construct your actual hardware drivers here.
     *
     * Replace these constructors with the real arguments required by
     * your drivers.
     */
    ImuDriver imu_driver{/* configuration */};
    DvlDriver dvl_driver{/* configuration */};
    PressureDriver pressure_driver{/* configuration */};
    ThrusterDriver thruster_driver{/* configuration */};

    HardwareIo hardware_io{
        HardwareIoConfig{
            .command_timeout =
                std::chrono::milliseconds{200},

            .max_thruster_force_n =
                vehicle_config.allocator.max_force,
        },
        imu_driver,
        dvl_driver,
        pressure_driver,
        thruster_driver,
    };

    RuntimeState runtime_state{
        .running = true,
        .killswitch_on = true,
        .operation_mode =
            vortex::utils::types::Mode::manual,
    };

    auto previous_time =
        std::chrono::steady_clock::now();

    while (running.load()) {
        const auto current_time =
            std::chrono::steady_clock::now();

        const double dt_s =
            std::chrono::duration<double>(
                current_time - previous_time)
                .count();

        previous_time = current_time;


        runtime_state.running = true;

        /*
         * These should come from whichever component currently owns
         * the killswitch and operation mode.
         */
        // runtime_state.killswitch_on =
        //     thruster_driver.killswitch_on();

        runtime_state.operation_mode =
            vortex::utils::types::Mode::autonomous;

        const SensorFrame sensors =
            hardware_io.read_sensors();

        const ControlOutput output =
            vehicle_control.tick(
                sensors,
                runtime_state,
                dt_s);

        if (!hardware_io.apply_thrusters(output)) {
            runtime_state.killswitch_on = true;
        }

        hardware_io.update_watchdog();

        std::this_thread::sleep_until(
            current_time + control_period);
    }

    runtime_state.running = false;

    static_cast<void>(
        hardware_io.stop_thrusters());

    return 0;
}
