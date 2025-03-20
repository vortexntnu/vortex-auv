from dataclasses import dataclass, field
import numpy as np
from eskf_python_utils import quaternion_error

@dataclass
class StateQuat:
    position: np.ndarray = field(
        default_factory=lambda: np.zeros(3)
    )  # Position vector (x, y, z)
    velocity: np.ndarray = field(
        default_factory=lambda: np.zeros(3)
    )  # Velocity vector (u, v, w)
    orientation: np.ndarray = field(
        default_factory=lambda: np.array([1, 0, 0, 0])
    )  # Orientation quaternion (w, x, y, z)
    acceleration_bias: np.ndarray = field(
        default_factory=lambda: np.zeros(3)
    )  # Acceleration bias vector (b_ax, b_ay, b_az)
    gyro_bias: np.ndarray = field(
        default_factory=lambda: np.zeros(3)
    )  # Gyro bias vector (b_gx, b_gy, b_gz)
    g: np.ndarray = field(
        default_factory=lambda: np.array([0, 0, 0])
    )  # Gravity vector
    
    def as_vector(self) -> np.ndarray:
        """Returns the state vector as a numpy array.

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
                self.g,
            ]
        )

    def fill_states(self, state: np.ndarray) -> None:
        """Fills the state vector with the values from a numpy array.

        Args:
            state (np.ndarray): The state vector.
        """
        self.position = state[0:3]
        self.velocity = state[3:6]
        self.orientation = state[6:10]
        self.acceleration_bias = state[10:13]
        self.gyro_bias = state[13:16]
        self.g = state[16:19]

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
    
    def __sub__(self, other: 'StateQuat') -> 'StateQuat':
        """Subtracts the values of two state vectors.

        Args:
            other (StateQuat): The state vector to subtract.

        Returns:
            np.ndarray: The difference between the two state vectors.
        """
        result = StateQuat()
        result.position = self.position - other.position
        result.velocity = self.velocity - other.velocity
        result.orientation = quaternion_error(self.orientation, other.orientation)
        result.acceleration_bias = self.acceleration_bias - other.acceleration_bias
        result.gyro_bias = self.gyro_bias - other.gyro_bias
        result.g = self.g - other.g

        return result


    
@dataclass
class StateEuler:
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
    g: np.ndarray = field(
        default_factory=lambda: np.array([0, 0, 9.81])
    )  # Gravity vector
    covariance: np.ndarray = field(
        default_factory=lambda: np.zeros((18, 18))
    )  # Covariance matrix

    def as_vector(self) -> np.ndarray:
        """Returns the state vector as a numpy array.

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
                self.g,
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
        self.g = state[15:18]

    def copy_state(self, wanted_state: 'StateEuler') -> None:
        """Copies the state from a StateVector object into the current StateVector object.

        Args:
            wanted_state (StateVector_euler): The quaternion state to copy from.
        """
        self.position = wanted_state.position
        self.velocity = wanted_state.velocity
        self.orientation = wanted_state.orientation
        self.acceleration_bias = wanted_state.acceleration_bias
        self.gyro_bias = wanted_state.gyro_bias


@dataclass
class MeasurementModel:
    measurement: np.ndarray = field(
        default_factory=lambda: np.zeros(6)
    )
    measurement_covariance: np.ndarray = field(
        default_factory=lambda: np.zeros((6, 6))
    )

    def H(self) -> np.ndarray:
        """Calculates the measurement matrix.

        Returns:
            np.ndarray: The measurement matrix.
        """
        H = np.zeros((3, 15))

        H[0:3, 3:6] = np.eye(3)

        return H
    
@dataclass
class Measurement:
    acceleration: np.ndarray = field(
        default_factory=lambda: np.zeros(3)
    )
    angular_velocity: np.ndarray = field(
        default_factory=lambda: np.zeros(3)
    )
    aiding: np.ndarray = field(
        default_factory=lambda: np.zeros(3)
    )

    aiding_covariance: np.ndarray = field(
        default_factory=lambda: np.zeros((3, 3))
    )