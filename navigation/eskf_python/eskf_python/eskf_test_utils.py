from dataclasses import dataclass, field

import numpy as np
from eskf_python_utils import (
    euler_to_quat,
    quat_norm,
    quat_to_euler,
    quaternion_error,
    quaternion_product,
    skew_matrix,
)

# This was the original code from the ukf_okid.py file


@dataclass
class StateQuatModel:
    """A class to represent the state to be estimated by the UKF.
    """

    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    orientation: np.ndarray = field(default_factory=lambda: np.array([1, 0, 0, 0]))
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    angular_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    covariance: np.ndarray = field(default_factory=lambda: np.zeros((12, 12)))

    def as_vector(self) -> np.ndarray:
        """Returns the StateVector as a numpy array."""
        return np.concatenate(
            [self.position, self.orientation, self.velocity, self.angular_velocity]
        )

    def nu(self) -> np.ndarray:
        """Calculates the nu vector."""
        return np.concatenate([self.velocity, self.angular_velocity])

    def R_q(self) -> np.ndarray:
        """Calculates the rotation matrix from the orientation quaternion."""
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

    def fill_states(self, state: np.ndarray) -> None:
        """Fills the state vector with the values from a numpy array."""
        self.position = state[0:3]
        self.orientation = state[3:7]
        self.velocity = state[7:10]
        self.angular_velocity = state[10:13]

    def fill_states_different_dim(
        self, state: np.ndarray, state_euler: np.ndarray
    ) -> None:
        """Fills states when the state vector has different dimensions than the default state vector."""
        self.position = state[0:3] + state_euler[0:3]
        self.orientation = quaternion_product(
            state[3:7], euler_to_quat(state_euler[3:6])
        )
        self.velocity = state[7:10] + state_euler[6:9]
        self.angular_velocity = state[10:13] + state_euler[9:12]

    def subtract(self, other: 'StateQuatModel') -> np.ndarray:
        """Subtracts two StateQuatModel objects, returning the difference with Euler angles."""
        new_array = np.zeros(len(self.as_vector()) - 1)
        new_array[:3] = self.position - other.position
        new_array[3:6] = quat_to_euler(
            quaternion_error(self.orientation, other.orientation)
        )
        new_array[6:9] = self.velocity - other.velocity
        new_array[9:12] = self.angular_velocity - other.angular_velocity

        return new_array

    def __add__(self, other: 'StateQuatModel') -> 'StateQuatModel':
        """Adds two StateQuatModel objects."""
        new_state = StateQuatModel()
        new_state.position = self.position + other.position
        new_state.orientation = quaternion_product(self.orientation, other.orientation)
        new_state.velocity = self.velocity + other.velocity
        new_state.angular_velocity = self.angular_velocity + other.angular_velocity

        return new_state

    def __sub__(self, other: 'StateQuatModel') -> 'StateQuatModel':
        """Subtracts two StateQuatModel objects."""
        new_state = StateQuatModel()
        new_state.position = self.position - other.position
        new_state.orientation = quaternion_error(self.orientation, other.orientation)
        new_state.velocity = self.velocity - other.velocity
        new_state.angular_velocity = self.angular_velocity - other.angular_velocity

        return new_state.as_vector()

    def __rmul__(self, scalar: float) -> 'StateQuatModel':
        """Multiplies the StateQuatModel object by a scalar."""
        new_state = StateQuatModel()
        new_state.position = scalar * self.position
        new_state.orientation = quat_norm(scalar * self.orientation)
        new_state.velocity = scalar * self.velocity
        new_state.angular_velocity = scalar * self.angular_velocity

        return new_state

    def insert_weights(self, weights: np.ndarray) -> np.ndarray:
        """Inserts the weights into the covariance matrix."""
        new_state = StateQuatModel()
        new_state.position = self.position - weights[:3]
        new_state.orientation = quaternion_error(
            self.orientation, euler_to_quat(weights[3:6])
        )
        new_state.velocity = self.velocity - weights[6:9]
        new_state.angular_velocity = self.angular_velocity - weights[9:12]

        return new_state.as_vector()

    def add_without_quaternions(self, other: 'StateQuatModel') -> None:
        """Adds elements into the state vector without considering the quaternions."""
        self.position += other.position
        self.velocity += other.velocity
        self.angular_velocity += other.angular_velocity


@dataclass
class process_model:
    """A class defined for a general process model.
    """

    state_vector: StateQuatModel = field(default_factory=StateQuatModel)
    state_vector_dot: StateQuatModel = field(default_factory=StateQuatModel)
    state_vector_prev: StateQuatModel = field(default_factory=StateQuatModel)
    Control_input: np.ndarray = field(default_factory=lambda: np.zeros(6))
    mass_interia_matrix: np.ndarray = field(default_factory=lambda: np.zeros((6, 6)))
    added_mass: np.ndarray = field(default_factory=lambda: np.zeros(6))
    damping_linear: np.ndarray = field(default_factory=lambda: np.zeros(6))
    damping_nonlinear: np.ndarray = field(default_factory=lambda: np.zeros(6))
    m: float = 0.0
    inertia: np.ndarray = field(default_factory=lambda: np.zeros((3, 3)))
    r_b_bg: np.ndarray = field(default_factory=lambda: np.zeros(3))
    dt: float = 0.0
    integral_error_position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    integral_error_orientation: np.ndarray = field(default_factory=lambda: np.zeros(4))
    prev_position_error: np.ndarray = field(default_factory=lambda: np.zeros(3))
    prev_orientation_error: np.ndarray = field(default_factory=lambda: np.zeros(3))

    def R(self) -> np.ndarray:
        """Calculates the rotation matrix."""
        nu, e_1, e_2, e_3 = self.state_vector.orientation
        R = np.array(
            [
                [
                    1 - 2 * e_2**2 - 2 * e_3**2,
                    2 * e_1 * e_2 - 2 * nu * e_3,
                    2 * e_1 * e_3 + 2 * nu * e_2,
                ],
                [
                    2 * e_1 * e_2 + 2 * nu * e_3,
                    1 - 2 * e_1**2 - 2 * e_3**2,
                    2 * e_2 * e_3 - 2 * nu * e_1,
                ],
                [
                    2 * e_1 * e_3 - 2 * nu * e_2,
                    2 * e_2 * e_3 + 2 * nu * e_1,
                    1 - 2 * e_1**2 - 2 * e_2**2,
                ],
            ]
        )
        return R

    def T(self) -> np.ndarray:
        """Calculates the transformation matrix."""
        nu, e_1, e_2, e_3 = self.state_vector.orientation
        T = 0.5 * np.array(
            [[-e_1, -e_2, -e_3], [nu, -e_3, e_2], [e_3, nu, -e_1], [-e_2, e_1, nu]]
        )
        return T

    def Crb(self) -> np.ndarray:
        """Calculates the Coriolis matrix."""
        ang_vel = self.state_vector.angular_velocity
        ang_vel_skew = skew_matrix(ang_vel)
        lever_arm_skew = skew_matrix(self.r_b_bg)
        Crb = np.zeros((6, 6))
        Crb[0:3, 0:3] = self.m * ang_vel_skew
        Crb[3:6, 3:6] = -skew_matrix(np.dot(self.inertia, ang_vel))
        Crb[0:3, 3:6] = -self.m * np.dot(ang_vel_skew, lever_arm_skew)
        Crb[3:6, 0:3] = self.m * np.dot(lever_arm_skew, ang_vel_skew)
        return Crb

    def D(self) -> np.ndarray:
        """Calculates the damping matrix."""
        D_l = -np.diag(self.damping_linear)
        D_nl = -np.diag(self.damping_nonlinear) * np.abs(self.state_vector.nu())
        return D_l + D_nl

    def model_prediction(self, state: StateQuatModel) -> None:
        """Calculates the model of the system."""
        self.state_vector = state
        self.state_vector_dot.position = np.dot(self.R(), self.state_vector.velocity)
        self.state_vector_dot.orientation = np.dot(
            self.T(), self.state_vector.angular_velocity
        )
        Nu = np.linalg.inv(self.mass_interia_matrix + np.diag(self.added_mass)) @ (
            self.Control_input
            - np.dot(self.Crb(), self.state_vector.nu())
            - np.dot(self.D(), self.state_vector.nu())
        )
        self.state_vector_dot.velocity = Nu[:3]
        self.state_vector_dot.angular_velocity = Nu[3:]

    def euler_forward(self) -> StateQuatModel:
        """Calculates the forward Euler integration."""
        self.state_vector.position = (
            self.state_vector_prev.position + self.state_vector_dot.position * self.dt
        )
        self.state_vector.orientation = quat_norm(
            self.state_vector_prev.orientation
            + self.state_vector_dot.orientation * self.dt
        )
        self.state_vector.velocity = (
            self.state_vector_prev.velocity + self.state_vector_dot.velocity * self.dt
        )
        self.state_vector.angular_velocity = (
            self.state_vector_prev.angular_velocity
            + self.state_vector_dot.angular_velocity * self.dt
        )
        return self.state_vector
