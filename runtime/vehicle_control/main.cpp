#include <chrono>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <thread>

#include <vortex/propulsion/thrust_allocator/thrust_allocator.hpp>
#include <vortex/propulsion/thruster_interface/thruster_interface.hpp>
#include "dp_adapt_backs_controller_quat/dp_adapt_backs_controller_core.hpp"
#include "reference_filter_dp_quat/waypoint_guidance_manager.hpp"

namespace {

volatile std::sig_atomic_t running = 1;

void handle_signal(int) {
    running = 0;
}

constexpr auto control_period = std::chrono::milliseconds{10};

}  // namespace

int main() {
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

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

    vortex::control::DPAdaptBacksControllerCoreConfig controller_config{
        .controller_params =
            {
                .adapt_param = Eigen::Vector12d::Zero(),
                .d_gain = Eigen::Vector6d::Ones(),
                .K1 = Eigen::Vector6d::Ones(),
                .K2 = Eigen::Vector6d::Ones(),
                .r_b_bg = Eigen::Vector3d::Zero(),

                .inertia_matrix_body = Eigen::Vector3d::Ones(),

                .mass_intertia_matrix = Eigen::Matrix6d::Identity(),

                .tau_max = Eigen::Vector6d::Constant(100.0),

                .mass = 10.0,

                .time_step_s =
                    static_cast<double>(control_period.count()) / 1000.0,

                .singularity_tolerance = 1e-6,
                .adapt_param_max = 100.0,
                .d_est_max = 100.0,
            },
        .time_step = control_period,
        .initial_killswitch = true,
        .initial_operation_mode = vortex::utils::types::Mode::manual,
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

    vortex::control::DPAdaptBacksControllerCore controller{controller_config};

    vortex::propulsion::ThrustAllocator allocator{allocator_settings};

    std::vector<vortex::propulsion::ThrusterParameters> thruster_parameters{
        {.mapping = 0, .direction = 1, .pwm_min = 1100, .pwm_max = 1900},
        {.mapping = 1, .direction = 1, .pwm_min = 1100, .pwm_max = 1900},
        {.mapping = 2, .direction = 1, .pwm_min = 1100, .pwm_max = 1900},
        {.mapping = 3, .direction = 1, .pwm_min = 1100, .pwm_max = 1900},
        {.mapping = 4, .direction = 1, .pwm_min = 1100, .pwm_max = 1900},
        {.mapping = 5, .direction = 1, .pwm_min = 1100, .pwm_max = 1900},
        {.mapping = 6, .direction = 1, .pwm_min = 1100, .pwm_max = 1900},
        {.mapping = 7, .direction = 1, .pwm_min = 1100, .pwm_max = 1900},
    };

    std::vector<double> right_coeffs{
        1500.0,
        100.0,
    };

    std::vector<double> left_coeffs{
        1500.0,
        100.0,
    };

    vortex::propulsion::ThrusterInterface thrusters{
        "can0",
        thruster_parameters,
        right_coeffs,
        left_coeffs,
    };

    vortex::utils::types::Pose pose{};
    vortex::utils::types::Twist twist{};

    bool killswitch_on = true;
    auto operation_mode = vortex::utils::types::Mode::manual;

    auto next_tick = std::chrono::steady_clock::now();

    while (running) {
        next_tick += control_period;

        guidance.set_pose(pose);
        guidance.set_twist(twist);

        // if (state_input.altitude_valid()) {
        //     guidance.set_altitude(state_input.altitude_m());
        // }

        const auto reference = guidance.tick();

        controller.set_pose(pose);
        controller.set_twist(twist);
        controller.set_guidance_pose(reference.pose);
        controller.set_operation_mode(operation_mode);
        controller.set_killswitch(killswitch_on);

        const auto wrench = controller.tick();

        if (!wrench) {
            // thrusters.send_zero();
            std::this_thread::sleep_until(next_tick);
            continue;
        }

        const auto forces = allocator.allocate_thrust(*wrench);

        if (!forces) {
            return -1;
        }

        const auto pwm = thrusters.drive_thrusters(*forces);

        if (!pwm) {
            return -1;
        }

        std::this_thread::sleep_until(next_tick);
    }

    // thrusters.send_zero();
    // thrusters.disable();

    return 0;
}
