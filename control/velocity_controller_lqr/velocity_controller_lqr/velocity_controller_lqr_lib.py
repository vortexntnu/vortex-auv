from dataclasses import dataclass

import numpy as np
from vortex_utils.python_utils import State, ssa

import control as ct


@dataclass
class LQRParameters:
    """Dataclass to store the parameters for the LQR controller.

    Attributes:
        q_surge: float: Surge state weight
        q_pitch: float: Pitch state weight
        q_yaw: float: Yaw state weight
        r_surge: float: Surge input weight
        r_pitch: float: Pitch input weight
        r_yaw: float: Yaw input weight
        i_surge: float: Surge integral weight
        i_pitch: float: Pitch integral weight
        i_yaw: float: Yaw integral weight
        i_weight: float: Integral weight
        max_force: float: Maximum force
    """

    q_surge: float = 0.0
    q_pitch: float = 0.0
    q_yaw: float = 0.0
    r_surge: float = 0.0
    r_pitch: float = 0.0
    r_yaw: float = 0.0
    i_surge: float = 0.0
    i_pitch: float = 0.0
    i_yaw: float = 0.0
    i_weight: float = 0.0
    max_force: float = 0.0
    operation_mode: str = "xbox mode"
    killswitch: bool = True


class LQRController:
    def __init__(self, parameters: LQRParameters, inertia_matrix: np.array) -> None:
        self.set_params(parameters)
        self.set_matrices(inertia_matrix)

    def saturate(self, value: float, windup: bool, limit: float) -> tuple:
        """Function to saturate the value within the limits, and set the windup flag.

        Parameters:
            value: float: value to be saturated
            windup: bool: windup variable
            limit: float: limit for saturation

        Returns:
            (value, windup): tuple: saturated value and windup flag

        """
        if abs(value) > limit:
            windup = True
            value = limit * (value / abs(value))
        else:
            windup = False

        return windup, value

    @staticmethod
    def anti_windup(
        k_i: float, error: float, integral_sum: float, windup: bool
    ) -> float:
        """Function to saturate the integral value within the limits.

        Parameters:
            k_i : float: integral gain
            error: float: error value
            integral_sum: float: integral sum
            windup: bool: windup variable

        Returns:
            value: float: saturated value

        """
        if not windup:
            integral_sum += k_i * error

        return integral_sum

    @staticmethod
    def calculate_coriolis_matrix(state: State) -> np.array:
        """Calculates the 3x3 coriolis matrix.

        Parameters:
            state: State: Current Pose and Twist values

        Returns:
            C: np.array: 3x3 Coriolis Matrix

        """
        return np.array(
            [
                [
                    0.2,
                    -30 * state.twist.linear_y * 0.01,
                    -30 * state.twist.linear_z * 0.01,
                ],
                [30 * state.twist.linear_y * 0.01, 0, 1.629 * state.twist.angular_y],
                [30 * state.twist.linear_z * 0.01, 1.769 * state.twist.angular_z, 0],
            ]
        )

    def set_params(self, parameters: LQRParameters) -> None:
        """Sets the parameters for the LQR controller.

        Parameters:
            parameters: LQRParameters: dataclass containing the parameters for the LQR controller

        """
        self.integral_error_surge = 0.0
        self.integral_error_pitch = 0.0
        self.integral_error_yaw = 0.0

        self.surge_windup = False
        self.pitch_windup = False
        self.yaw_windup = False

        self.q_surge = parameters.q_surge
        self.q_pitch = parameters.q_pitch
        self.q_yaw = parameters.q_yaw

        self.r_surge = parameters.r_surge
        self.r_pitch = parameters.r_pitch
        self.r_yaw = parameters.r_yaw

        self.i_surge = parameters.i_surge
        self.i_pitch = parameters.i_pitch
        self.i_yaw = parameters.i_yaw
        self.i_weight = parameters.i_weight
        self.max_force = parameters.max_force

        self.operation_mode = parameters.operation_mode
        self.killswitch = parameters.killswitch

    def set_matrices(self, inertia_matrix: np.array) -> None:
        """Adjusts the matrices for the LQR controller.

        Parameters:
            inertia_matrix: np.array: 3x3 inertia matrix
        """
        self.inertia_matrix_inv = np.linalg.inv(inertia_matrix)
        self.state_weight_matrix = np.block(
            [
                [np.diag([self.q_surge, self.q_pitch, self.q_yaw]), np.zeros((3, 3))],
                [
                    np.zeros((3, 3)),
                    np.diag([self.i_weight, self.i_weight, self.i_weight]),
                ],
            ]
        )

        self.input_weight_matrix = np.diag([self.r_surge, self.r_pitch, self.r_yaw])

    def update_augmented_matrices(self, coriolis_matrix: np.array) -> None:
        """Updates the augmented matrices for the LQR controller.

        Parameters:
            coriolis_matrix: np.array: 3x3 Coriolis Matrix
        """
        system_matrix = self.inertia_matrix_inv @ coriolis_matrix
        input_matrix = self.inertia_matrix_inv

        self.augmented_system_matrix = np.block(
            [[system_matrix, np.zeros((3, 3))], [-np.eye(3), np.zeros((3, 3))]]
        )
        self.augmented_input_matrix = np.block([[input_matrix], [np.zeros((3, 3))]])

    def update_error(self, guidance_values: State, states: State) -> np.array:
        """Updates the error values for the LQR controller.

        Parameters:
            guidance_values: State: Desired Pose and Twist values
            state: State: Current Pose and Twist values

        Returns:
            state_error: np.array: 6x1 array of the state errors
        """
        surge_error = (
            guidance_values.twist.linear_x - states.twist.linear_x
        )  # Surge error isn't an angle, no need for angle wrapping
        pitch_error = ssa(guidance_values.pose.pitch - states.pose.pitch)
        yaw_error = ssa(guidance_values.pose.yaw - states.pose.yaw)

        # Update the running integrator sums
        self.integral_error_surge = self.anti_windup(
            self.i_surge, surge_error, self.integral_error_surge, self.surge_windup
        )
        self.integral_error_pitch = self.anti_windup(
            self.i_pitch, pitch_error, self.integral_error_pitch, self.pitch_windup
        )
        self.integral_error_yaw = self.anti_windup(
            self.i_yaw, yaw_error, self.integral_error_yaw, self.yaw_windup
        )

        state_error = np.array(
            [
                -surge_error,
                -pitch_error,
                -yaw_error,
                self.integral_error_surge,
                self.integral_error_pitch,
                self.integral_error_yaw,
            ]
        )
        return state_error

    def saturate_input(self, u: np.array) -> np.array:
        """Saturates the control input within the limits, and sets the windup flag.

        Parameters:
            u: np.array: 3x1 control input

        Returns:
            u: np.array: 3x1 saturated control input
        """
        self.surge_windup, force_x = self.saturate(
            u[0], self.surge_windup, self.max_force
        )
        self.pitch_windup, torque_y = self.saturate(
            u[1], self.pitch_windup, self.max_force
        )
        self.yaw_windup, torque_z = self.saturate(u[2], self.yaw_windup, self.max_force)

        return np.array([force_x, torque_y, torque_z])

    def calculate_lqr_u(self, state: State, guidance_values: State) -> np.array:
        """Calculates the control input using the control library in python.

        Parameters:
            state: State: Current Pose and Twist values
            guidance_values: State: Desired Pose and Twist values

        Returns:
            u: np.array: 3x1 control input

        """
        coriolis_matrix = self.calculate_coriolis_matrix(state)
        self.update_augmented_matrices(coriolis_matrix)

        lqr_gain, _, _ = ct.lqr(
            self.augmented_system_matrix,
            self.augmented_input_matrix,
            self.state_weight_matrix,
            self.input_weight_matrix,
        )

        state_error = self.update_error(guidance_values, state)
        u = self.saturate_input(-lqr_gain @ state_error)

        return u

    def reset_controller(self) -> None:
        """Resets the controller to the initial state."""
        self.integral_error_surge = 0.0
        self.integral_error_pitch = 0.0
        self.integral_error_yaw = 0.0

        self.surge_windup = False
        self.pitch_windup = False
        self.yaw_windup = False


#                     .--------------.
#                 .---'  o        .    `---.
#              .-'    .    O  .         .   `-.
#           .-'     @@@@@@       .             `-.
#         .'@@   @@@@@@@@@@@       @@@@@@@   .    `.
#       .'@@@  @@@@@@@@@@@@@@     @@@@@@@@@         `.
#      /@@@  o @@@@@@@@@@@@@@     @@@@@@@@@     O     \
#     /        @@@@@@@@@@@@@@  @   @@@@@@@@@ @@     .  \
#    /@  o      @@@@@@@@@@@   .  @@  @@@@@@@@@@@     @@ \
#   /@@@      .   @@@@@@ o       @  @@@@@@@@@@@@@ o @@@@ \
#  /@@@@@                  @ .      @@@@@@@@@@@@@@  @@@@@ \
#  |@@@@@    O    `.-./  .        .  @@@@@@@@@@@@@   @@@  |
# / @@@@@        --`-'       o        @@@@@@@@@@@ @@@    . \
# |@ @@@@ .  @  @    `    @            @@      . @@@@@@    |
# |   @@                         o    @@   .     @@@@@@    |
# |  .     @   @ @       o              @@   o   @@@@@@.   |
# \     @    @       @       .-.       @@@@       @@@      /
#  |  @    @  @              `-'     . @@@@     .    .    |
#  \ .  o       @  @@@@  .              @@  .           . /
#   \      @@@    @@@@@@       .                   o     /
#    \    @@@@@   @@\@@    /        O          .        /
#     \ o  @@@       \ \  /  __        .   .     .--.  /
#      \      .     . \.-.---                   `--'  /
#       `.             `-'      .                   .'
#         `.    o     / | `           O     .     .'
#           `-.      /  |        o             .-'
#              `-.          .         .     .-'
#                 `---.        .       .---'
#                      `--------------'
