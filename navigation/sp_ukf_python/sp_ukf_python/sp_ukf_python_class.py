from dataclasses import dataclass, field

import numpy as np
from sp_ukf_python_utils import (
    euler_rotation_quaternion,
    quaternion_error,
    quaternion_super_product,
    ssa,
    quat_norm,
)


@dataclass
class StateVector_quaternion:
    position: np.ndarray = field(
        default_factory=lambda: np.zeros(3)
    )  # Position vector (x, y, z)
    velocity: np.ndarray = field(
        default_factory=lambda: np.zeros(3)
    )  # Velocity vector (u, v, w)
    orientation: np.ndarray = field(
        default_factory=lambda: np.zeros(4)
    )  # Orientation quaternion (w, x, y, z)
    acceleration_bias: np.ndarray = field(
        default_factory=lambda: np.zeros(3)
    )  # Acceleration bias vector (b_ax, b_ay, b_az)
    gyro_bias: np.ndarray = field(
        default_factory=lambda: np.zeros(3)
    )  # Gyro bias vector (b_gx, b_gy, b_gz)

    def as_vector(self) -> np.ndarray:
        """Calculates the state vector.

        Returns:
            np.ndarray: The state vector.
        """
        return np.concatenate(
            [
                self.position,
                self.velocity,
                self.orientation,
                self.acceleration_bias,
                self.gyro_bias,
            ]
        )

    def fill_states(self, state: np.ndarray) -> None:
        """Fills the state vector with the values from a numpy array.

        Args:
            state (np.ndarray): The state vector.
        """
        if len(state) == 15:
            self.position = state[0:3]
            self.velocity = state[3:6]
            self.orientation = state[6:10]
            self.acceleration_bias = state[10:13]
            self.gyro_bias = state[13:]
        else:
            self.position = state[0:3]
            self.velocity = state[3:6]
            self.orientation = euler_rotation_quaternion(state[6:9])
            self.acceleration_bias = state[9:12]
            self.gyro_bias = state[12:]

    def R_q(self) -> np.ndarray:
        """Calculates the rotation matrix from the orientation quaternion.

        Returns:
            np.ndarray: The rotation matrix.
        """
        q0, q1, q2, q3 = self.orientation
        R = np.array(
            [
                [
                    1 - 2 * q2**2 - 2 * q3**2,
                    2 * (q1 * q2 - q0 * q3),
                    2 * (q0 * q2 + q1 * q3),
                ],
                [
                    2 * (q1 * q2 + q0 * q3),
                    1 - 2 * q1**2 - 2 * q3**2,
                    2 * (q2 * q3 - q0 * q1),
                ],
                [
                    2 * (q1 * q3 - q0 * q2),
                    2 * (q0 * q1 + q2 * q3),
                    1 - 2 * q1**2 - 2 * q2**2,
                ],
            ]
        )

        return R

    def euler_forward(
        self, current_state: 'StateVector_quaternion', dt: float
    ) -> 'StateVector_quaternion':
        # Define the new state
        new_state = StateVector_quaternion()

        # Define the state derivatives
        new_state.position = current_state.position + self.position * dt
        new_state.velocity = current_state.velocity + self.velocity * dt
        new_state.orientation = quat_norm(current_state.orientation + self.orientation * dt)
        new_state.acceleration_bias = (
            current_state.acceleration_bias + self.acceleration_bias * dt
        )
        new_state.gyro_bias = current_state.gyro_bias + self.gyro_bias * dt

        # Normalize the orientation quaternion
        new_state.orientation /= np.linalg.norm(new_state.orientation)

        return new_state

    def __sub__(self, other: 'StateVector_quaternion') -> np.ndarray:
        """Subtracts two StateVector_quaternion objects.

        Args:
            other (StateVector_quaternion): The other StateVector_quaternion object.

        Returns:
            np.ndarray: The difference between the two StateVector_quaternion objects.
        """
        position_diff = self.position - other.position
        velocity_diff = self.velocity - other.velocity
        orientation_diff = quaternion_error(self.orientation, other.orientation)
        acceleration_bias_diff = self.acceleration_bias - other.acceleration_bias
        gyro_bias_diff = self.gyro_bias - other.gyro_bias

        return np.concatenate(
            [
                position_diff,
                velocity_diff,
                orientation_diff,
                acceleration_bias_diff,
                gyro_bias_diff,
            ]
        )

    def __add__(self, other: 'np.ndarray') -> 'np.ndarray':
        """Adds a numpy array to this StateVector_quaternion.

        Args:
            other (np.ndarray): The numpy array to add.

        Returns:
            np.ndarray: The result of the addition.
        """
        # Construct the quaternion from the array
        add_to_position = other[:3]
        add_to_orientation = euler_rotation_quaternion(other[6:10])

        new_position = self.position + add_to_position
        new_velcoity = self.velocity + other[3:6]
        new_orientation = quaternion_super_product(self.orientation, add_to_orientation)
        new_acceleration_bias = self.acceleration_bias + other[10:13]
        new_gyro_bias = self.gyro_bias + other[13:]

        return np.concatenate(
            [
                new_position,
                new_velcoity,
                new_orientation,
                new_acceleration_bias,
                new_gyro_bias,
            ]
        )


@dataclass
class StateVector_euler:
    position: np.ndarray = field(
        default_factory=lambda: np.zeros(3)
    )  # Position vector (x, y, z)
    velocity: np.ndarray = field(
        default_factory=lambda: np.zeros(3)
    )  # Velocity vector (u, v, w)
    orientation: np.ndarray = field(
        default_factory=lambda: np.zeros(3)
    )  # Orientation angles (roll, pitch, yaw)
    acceleration_bias: np.ndarray = field(
        default_factory=lambda: np.zeros(3)
    )  # Acceleration bias vector (b_ax, b_ay, b_az)
    gyro_bias: np.ndarray = field(
        default_factory=lambda: np.zeros(3)
    )  # Gyro bias vector (b_gx, b_gy, b_gz)
    covariance: np.ndarray = field(
        default_factory=lambda: np.zeros((15, 15))
    )  # Covariance matrix

    def as_vector(self) -> np.ndarray:
        """Calculates the state estimate vector.

        Returns:
            np.ndarray: The state estimate vector.
        """
        return np.concatenate(
            [
                self.position,
                self.velocity,
                self.orientation,
                self.acceleration_bias,
                self.gyro_bias,
            ]
        )

    def fill_states(self, state: np.ndarray) -> None:
        """Fills the state vector with the values from a numpy array.

        Args:
            state (np.ndarray): The state vector.
        """
        self.position = state[0:3]
        self.velocity = state[3:6]
        self.orientation = state[6:9]
        self.acceleration_bias = state[9:12]
        self.gyro_bias = state[12:15]

    def copy_state(self, wanted_state: 'StateVector_euler') -> None:
        """Copies the state from a StateVector object into the current StateVector object.

        Args:
            wanted_state (StateVector_euler): The quaternion state to copy from.
        """
        self.position = wanted_state.position
        self.velocity = wanted_state.velocity
        self.orientation = wanted_state.orientation
        self.acceleration_bias = wanted_state.acceleration_bias
        self.gyro_bias = wanted_state.gyro_bias

    def __add__(self, other: 'np.ndarray') -> 'np.ndarray':
        """Adds a numpy array to this StateVector_quaternion.

        Args:
            other (np.ndarray): The numpy array to add.

        Returns:
            np.ndarray: The result of the addition.
        """
        new_position = self.position + other[:3]
        new_velcoity = self.velocity + other[3:6]
        new_orientation = self.orientation + other[6:9]
        new_acceleration_bias = self.acceleration_bias + other[9:12]
        new_gyro_bias = self.gyro_bias + other[12:]

        return np.concatenate(
            [
                new_position,
                new_velcoity,
                new_orientation,
                new_acceleration_bias,
                new_gyro_bias,
            ]
        )

    def __sub__(self, other_state: 'StateVector_euler') -> 'StateVector_euler':
        """Subtracts two StateVector_euler objects.

        Args:
            other (StateVector_euler): The other StateVector_euler object.

        Returns:
            StateVector_euler: The difference between the two StateVector_euler objects.
        """
        position_diff = self.position - other_state[:3]
        velocity_diff = self.velocity - other_state[3:6]
        orientation_diff = ssa(self.orientation - other_state[6:9])
        acceleration_bias_diff = self.acceleration_bias - other_state[9:12]
        gyro_bias_diff = self.gyro_bias - other_state[12:]

        return np.concatenate(
            [
                position_diff,
                velocity_diff,
                orientation_diff,
                acceleration_bias_diff,
                gyro_bias_diff,
            ]
        )


@dataclass
class MeasurementModel:
    measurement: np.ndarray = field(
        default_factory=lambda: np.zeros(6)
    )  # Measurement vector
    measurement_matrix: np.ndarray = field(
        default_factory=lambda: np.zeros((6, 15))
    )  # Measurement matrix
    measurement_covariance: np.ndarray = field(
        default_factory=lambda: np.zeros((6, 6))
    )  # Measurement noise matrix
