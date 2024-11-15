#!/usr/bin/env python3
import numpy as np

import control as ct


class LQRController:
    def __init__(self, parameters, inertia_matrix):
        self.integral_error_surge = 0.0
        self.integral_error_pitch = 0.0
        self.integral_error_yaw = 0.0

        self.surge_windup = False  # Windup variable
        self.pitch_windup = False  # Windup variable
        self.yaw_windup = False  # Windup variable

        self.q_surge = parameters[0]  # Surge cost
        self.q_pitch = parameters[1]  # Pitch cost
        self.q_yaw = parameters[2]  # Yaw cost

        self.r_surge = parameters[3]  # Surge control cost
        self.r_pitch = parameters[4]  # Pitch control cost
        self.r_yaw = parameters[5]  # Yaw control cost

        self.i_surge = parameters[6]  # Integral gain for surge
        self.i_pitch = parameters[7]  # Integral gain for pitch
        self.i_yaw = parameters[8]  # Integral gain for yaw

        self.i_weight = parameters[9]  # Weight for integral gain

        self.max_force = parameters[10]  # Maximum force that can be applied

        self.inertia_matrix_inv = inertia_matrix  # Inverse of the inertia matrix

        # VARIABLES
        self.inertia_matrix_inv = np.linalg.inv(
            inertia_matrix
        )  # Inverse of the inertia matrix

        self.state_weight_matrix = np.block(
            [
                [np.diag([self.q_surge, self.q_pitch, self.q_yaw]), np.zeros((3, 3))],
                [
                    np.zeros((3, 3)),
                    np.diag([self.i_weight, self.i_weight, self.i_weight]),
                ],
            ]
        )  # Augmented state cost matrix
        self.input_weight_matrix = np.diag(
            [self.r_surge, self.r_pitch, self.r_yaw]
        )  # control cost matrix

    @staticmethod
    def quaternion_to_euler_angle(w: float, x: float, y: float, z: float) -> tuple:
        """Function to convert quaternion to euler angles.

        Args:
            w: float: w component of quaternion
            x: float: x component of quaternion
            y: float: y component of quaternion
            z: float: z component of quaternion

        Returns:
            X: float: roll angle
            Y: float: pitch angle
            Z: float: yaw angle

        """
        y_square = y * y

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y_square)
        phi = np.arctan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        theta = np.arcsin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y_square + z * z)
        psi = np.arctan2(t3, t4)

        return phi, theta, psi

    @staticmethod
    def ssa(angle: float) -> float:
        """Function to convert the angle to the smallest signed angle.

        Args:
            angle: float: angle in radians

        Returns:
            angle: float: angle in radians

        """
        if angle > np.pi:
            angle -= 2 * np.pi
        elif angle < -np.pi:
            angle += 2 * np.pi
        return angle

    def saturate(self, value: float, windup: bool, limit: float) -> tuple:
        """Function to saturate the value within the limits, and set the windup flag.

        Args:
            value: float: value to be saturated
            windup: bool: windup variable
            limit: float: limit for saturation

        Returns:
            (value, windup): tuple: saturated value and windup flag

        """
        if abs(value) > limit:
            windup = True
            value = limit * value / abs(value)
        else:
            windup = False

        return value, windup

    @staticmethod
    def anti_windup(
        k_i: float, error: float, integral_sum: float, windup: bool
    ) -> float:
        """Function to saturate the integral value within the limits.

        Args:
            k_i : float: integral gain
            error: float: error value
            integral_sum: float: integral sum
            windup: bool: windup variable



        Returns:
            value: float: saturated value

        """
        if windup:
            integral_sum += 0.0
        else:
            integral_sum += k_i * error

        return integral_sum

    def calculate_coriolis_matrix(
        self, pitch_rate: float, yaw_rate: float, sway: float, heave: float
    ) -> np.array:
        """Calculates the 3x3 coriolis matrix.

        Parameters:
            pitch_rate: float: pitch rate in rad/s
            yaw_rate: float: yaw rate in rad/s
            sway: float: sway velocity in m/s
            heave: float: heave velocity in m/s

        Returns:
            C: np.array: 3x3 Coriolis Matrix

        """
        return np.array(
            [
                [0.2, -30 * sway * 0.01, -30 * heave * 0.01],
                [30 * sway * 0.01, 0, 1.629 * pitch_rate],
                [30 * heave * 0.01, 1.769 * yaw_rate, 0],
            ]
        )

    def calculate_lqr_u(
        self, coriolis_matrix: np.array, states: np.array, guidance_values: np.array
    ) -> np.array:
        """Calculates the control input using the control library in python.

        Parameters:
            C: np.array: 3x3 Coriolis Matrix
            states: np.array: 6x1 state vector
            guidance_values: np.array: 6x1 guidance vector

        Returns:
            u: np.array: 3x1 control input

        """
        system_matrix = self.inertia_matrix_inv @ coriolis_matrix
        input_matrix = self.inertia_matrix_inv

        # Augment the A and B matrices for integrators for surge, pitch, and yaw
        self.augmented_system_matrix = np.block(
            [[system_matrix, np.zeros((3, 3))], [-np.eye(3), np.zeros((3, 3))]]
        )  # Integrators added for surge, pitch, and yaw
        self.augmented_input_matrix = np.block(
            [[input_matrix], [np.zeros((3, 3))]]
        )  # Control input does not affect integrators directly

        # CT LQR controller from control library python
        lqr_gain, ricatti_s, ricatti_e = ct.lqr(
            self.augmented_system_matrix,
            self.augmented_input_matrix,
            self.state_weight_matrix,
            self.input_weight_matrix,
        )

        # Calculate the control input
        surge_error = (
            guidance_values[0] - states[0]
        )  # Surge error no need for angle wrapping
        pitch_error = self.ssa(
            guidance_values[1] - states[1]
        )  # Apply ssa to pitch error
        yaw_error = self.ssa(guidance_values[2] - states[2])  # Apply ssa to yaw error

        # Update the integrators for surge, pitch, and yaw
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

        # Augmented state vector including integrators
        u = -lqr_gain @ state_error

        self.surge_windup, force_x = self.saturate(
            u[0], self.surge_windup, self.max_force
        )
        self.pitch_windup, torque_y = self.saturate(
            u[1], self.pitch_windup, self.max_force
        )
        self.yaw_windup, torque_z = self.saturate(u[2], self.yaw_windup, self.max_force)

        u = np.array([force_x, torque_y, torque_z])

        return u


# ---------------------------------------------------------------Main Function---------------------------------------------------------------


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
