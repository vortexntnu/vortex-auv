#include "nautilus_config.hpp"

#include <chrono>

namespace vortex::runtime::vehicle_control::config {

VehicleConfig make_nautilus_config(
    const std::chrono::milliseconds control_period
) {
    constexpr Eigen::Index num_thrusters = 8;

    const double time_step_s =
        std::chrono::duration<double>(control_period).count();

    /*
     * Thruster columns correspond to thrusters 0 through 7.
     *
     * Rows:
     *   0: force direction in body X
     *   1: force direction in body Y
     *   2: force direction in body Z
     */
    Eigen::Matrix<double, 3, num_thrusters> thruster_force_directions;

    thruster_force_directions <<
         0.70711, 0.0, 0.0, -0.70711, -0.70711, 0.0, 0.0, 0.70711,
        -0.70711, 0.0, 0.0, -0.70711,  0.70711, 0.0, 0.0, 0.70711,
         0.0,     1.0, 1.0,  0.0,      0.0,     1.0, 1.0, 0.0;

    /*
     * Thruster positions in the body frame.
     *
     * Rows:
     *   0: X position
     *   1: Y position
     *   2: Z position
     */
    Eigen::Matrix<double, 3, num_thrusters> thruster_positions;

    thruster_positions <<
         0.450150,  0.240600, -0.229700, -0.438610,
        -0.438610, -0.229700,  0.240600,  0.450150,

         0.305680,  0.313022,  0.313022,  0.305680,
        -0.305680, -0.313022, -0.313022, -0.305680,

         0.021736,  0.021736,  0.021736,  0.021736,
         0.021736,  0.021736,  0.021736,  0.021736;

    Eigen::Matrix6d mass_inertia_matrix;

    mass_inertia_matrix <<
        53.7, 0.0,  0.0,  0.0,      0.0,      0.0,
        0.0, 53.7,  0.0,  0.0,      0.0,      0.0,
        0.0, 0.0,  53.7,  0.0,      0.0,      0.0,
        0.0, 0.0,   0.0, 11.0628,   1.086,   -3.17502,
        0.0, 0.0,   0.0,  1.086,   23.1128,   0.1025,
        0.0, 0.0,   0.0, -3.17502,  0.1025,  26.23998;

    vortex::guidance::WaypointGuidanceManagerConfig guidance{
        .filter_params =
            {
                .omega = Eigen::Vector6d::Constant(1.0),
                .zeta = Eigen::Vector6d::Constant(1.0),
            },
        .time_step = control_period,
        .altitude_control_enabled = false,
        .altitude_low_pass_alpha = 0.9,
    };

    vortex::control::DPAdaptParams controller{
        .adapt_param =
            (Eigen::Vector12d() <<
                0.4, 0.2,
                0.4, 0.2,
                0.4, 0.2,
                0.4, 0.2,
                0.4, 0.2,
                0.4, 0.2)
                .finished(),

        .d_gain = Eigen::Vector6d::Constant(0.3),

        .K1 =
            (Eigen::Vector6d() <<
                4.0, 4.0, 12.5,
                1.5, 3.0, 3.0)
                .finished(),

        .K2 =
            (Eigen::Vector6d() <<
                25.0, 25.0, 50.0,
                25.0, 50.0, 50.0)
                .finished(),

        .r_b_bg = Eigen::Vector3d{
            0.0,
            0.0,
            0.015,
        },

        // Verify whether this member represents only rotational inertia.
        .inertia_matrix_body = Eigen::Vector3d{
            11.0628,
            23.1128,
            26.23998,
        },

        .mass_intertia_matrix = mass_inertia_matrix,

        .tau_max = Eigen::Vector6d::Constant(100.0),

        .mass = 53.7,

        .time_step_s = time_step_s,

        .singularity_tolerance = 1.0e-8,
        .adapt_param_max = 20.0,
        .d_est_max = 20.0,
    };

    vortex::propulsion::ThrustAllocatorSettings allocator{
        .solver_type = "pseudoinverse",

        .center_of_mass = Eigen::Vector3d{
            0.0,
            0.0,
            0.025,
        },

        .thruster_force_direction = thruster_force_directions,
        .thruster_position = thruster_positions,

        .input_weights = Eigen::VectorXd::Ones(num_thrusters),

        .slack_weights =
            Eigen::Vector6d::Constant(2000.0),

        .min_force = -41.0,
        .max_force = 39.0,
    };

    const Eigen::Vector12d q_std{
        0.05,   0.05,   0.1,
        0.01,   0.01,   0.02,
        0.001,  0.001,  0.001,
        0.0001, 0.0001, 0.0001,
    };

    const Eigen::Vector15d p_init{
        1.0,   1.0,   1.0,
        0.5,   0.5,   0.5,
        0.1,   0.1,   0.1,
        0.001, 0.001, 0.001,
        0.001, 0.001, 0.001,
    };

    EskfParams eskf{};

    eskf.Q = q_std.array().square().matrix().asDiagonal();

    // Preserve the old implementation's interpretation here.
    // Square this vector only if the old ESKF treated diag_p_init as std.
    eskf.P = p_init.asDiagonal();

    eskf.g_ = Eigen::Vector3d{
        0.0,
        0.0,
        9.82841,
    };

    const Eigen::Matrix3d dvl_measurement_noise =
        Eigen::Vector3d{
            0.1 * 0.1,
            0.1 * 0.1,
            0.1 * 0.1,
        }
            .asDiagonal();

    constexpr double pressure_measurement_noise_pa2 = 40000.0;

    return VehicleConfig{
        .guidance = std::move(guidance),
        .controller = std::move(controller),
        .allocator = std::move(allocator),
        .eskf = std::move(eskf),
        .dvl_measurement_noise = dvl_measurement_noise,
        .pressure_measurement_noise_pa2 =
            pressure_measurement_noise_pa2,
    };
}

}  // namespace vortex::runtime::vehicle_control::config

