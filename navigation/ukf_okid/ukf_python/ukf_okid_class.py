from dataclasses import dataclass, field

import numpy as np


@dataclass
class StateQuat:
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
        self.orientation = quaternion_super_product(
            state[3:7], euler_to_quat(state_euler[3:6])
        )
        self.velocity = state[7:10] + state_euler[6:9]
        self.angular_velocity = state[10:13] + state_euler[9:12]

    def subtract(self, other: 'StateQuat', error_ori: 'np.ndarray') -> np.ndarray:
        """Subtracts two StateQuat objects, returning the difference with Euler angles."""
        new_array = np.zeros(len(self.as_vector()) - 1)
        new_array[:3] = self.position - other.position
        new_array[3:6] = error_ori
        new_array[6:9] = self.velocity - other.velocity
        new_array[9:12] = self.angular_velocity - other.angular_velocity

        return new_array

    def __add__(self, other: 'StateQuat') -> 'StateQuat':
        """Adds two StateQuat objects."""
        new_state = StateQuat()
        new_state.position = self.position + other.position
        new_state.orientation = quaternion_super_product(
            self.orientation, other.orientation
        )
        new_state.velocity = self.velocity + other.velocity
        new_state.angular_velocity = self.angular_velocity + other.angular_velocity

        return new_state

    def __sub__(self, other: 'StateQuat') -> 'StateQuat':
        """Subtracts two StateQuat objects."""
        new_state = StateQuat()
        new_state.position = self.position - other.position
        new_state.orientation = quaternion_error(self.orientation, other.orientation)
        new_state.velocity = self.velocity - other.velocity
        new_state.angular_velocity = self.angular_velocity - other.angular_velocity

        return new_state.as_vector()

    def __rmul__(self, scalar: float) -> 'StateQuat':
        """Multiplies the StateQuat object by a scalar."""
        new_state = StateQuat()
        new_state.position = scalar * self.position
        new_state.orientation = quat_norm(scalar * self.orientation)
        new_state.velocity = scalar * self.velocity
        new_state.angular_velocity = scalar * self.angular_velocity

        return new_state

    def insert_weights(self, weights: np.ndarray) -> np.ndarray:
        """Inserts the weights into the covariance matrix."""
        new_state = StateQuat()
        new_state.position = self.position - weights[:3]
        new_state.orientation = quaternion_error(
            self.orientation, euler_to_quat(weights[3:6])
        )
        new_state.velocity = self.velocity - weights[6:9]
        new_state.angular_velocity = self.angular_velocity - weights[9:12]

        return new_state.as_vector()

    def add_without_quaternions(self, other: 'StateQuat') -> None:
        """Adds elements into the state vector without considering the quaternions."""
        self.position += other.position
        self.velocity += other.velocity
        self.angular_velocity += other.angular_velocity


@dataclass
class MeasModel:
    """A class defined for a general measurement model.
    """

    measurement: np.ndarray = field(default_factory=lambda: np.zeros(3))
    covariance: np.ndarray = field(default_factory=lambda: np.zeros((3, 3)))

    def H(self, state: StateQuat) -> 'MeasModel':
        """Calculates the measurement matrix."""
        H = np.zeros((3, 13))
        H[0:3, 7:10] = np.eye(3)
        z_i = MeasModel()
        z_i.measurement = np.dot(H, state.as_vector())
        return z_i

    def __add__(self, other: 'MeasModel') -> 'MeasModel':
        """Defines the addition operation between two MeasModel objects."""
        result = MeasModel()
        result.measurement = self.measurement + other.measurement
        return result

    def __rmul__(self, scalar: float) -> 'MeasModel':
        """Defines multiplication between scalar value and MeasModel object."""
        result = MeasModel()
        result.measurement = scalar * self.measurement
        return result

    def __sub__(self, other: 'MeasModel') -> 'MeasModel':
        """Defines the subtraction between two MeasModel objects."""
        result = MeasModel()
        result.measurement = self.measurement - other.measurement
        return result


@dataclass
class process_model:
    """A class defined for a general process model.
    """

    state_vector: StateQuat = field(default_factory=StateQuat)
    state_vector_dot: StateQuat = field(default_factory=StateQuat)
    state_vector_prev: StateQuat = field(default_factory=StateQuat)
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
        ang_vel_skew = skew_symmetric(ang_vel)
        lever_arm_skew = skew_symmetric(self.r_b_bg)
        Crb = np.zeros((6, 6))
        Crb[0:3, 0:3] = self.m * ang_vel_skew
        Crb[3:6, 3:6] = -skew_symmetric(np.dot(self.inertia, ang_vel))
        Crb[0:3, 3:6] = -self.m * np.dot(ang_vel_skew, lever_arm_skew)
        Crb[3:6, 0:3] = self.m * np.dot(lever_arm_skew, ang_vel_skew)
        return Crb

    def D(self) -> np.ndarray:
        """Calculates the damping matrix."""
        D_l = -np.diag(self.damping_linear)
        D_nl = -np.diag(self.damping_nonlinear) * np.abs(self.state_vector.nu())
        return D_l + D_nl

    def model_prediction(self, state: StateQuat) -> None:
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

    def euler_forward(self) -> StateQuat:
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


def euler_to_quat(euler_angles: np.ndarray) -> np.ndarray:
    """Converts Euler angles to a quaternion
    """
    psi, theta, phi = euler_angles
    c_psi = np.cos(psi / 2)
    s_psi = np.sin(psi / 2)
    c_theta = np.cos(theta / 2)
    s_theta = np.sin(theta / 2)
    c_phi = np.cos(phi / 2)
    s_phi = np.sin(phi / 2)

    quat = np.array(
        [
            c_psi * c_theta * c_phi + s_psi * s_theta * s_phi,
            c_psi * c_theta * s_phi - s_psi * s_theta * c_phi,
            s_psi * c_theta * s_phi + c_psi * s_theta * c_phi,
            s_psi * c_theta * c_phi - c_psi * s_theta * s_phi,
        ]
    )

    return quat


def quat_to_euler(quat: np.ndarray) -> np.ndarray:
    """Converts a quaternion to Euler angles
    """
    nu, eta_1, eta_2, eta_3 = quat

    phi = np.arctan2(2 * (eta_2 * eta_3 + nu * eta_1), 1 - 2 * (eta_1**2 + eta_2**2))
    theta = -np.arcsin(2 * (eta_1 * eta_3 - nu * eta_2))
    psi = np.arctan2(2 * (nu * eta_3 + eta_1 * eta_2), 1 - 2 * (eta_2**2 + eta_3**2))

    return np.array([phi, theta, psi])


def quat_norm(quat: np.ndarray) -> np.ndarray:
    """Function that normalizes a quaternion
    """
    quat = quat / np.linalg.norm(quat)

    return quat


def skew_symmetric(vector: np.ndarray) -> np.ndarray:
    """Calculates the skew symmetric matrix of a vector.

    Args:
        vector (np.ndarray): The vector.

    Returns:
        np.ndarray: The skew symmetric matrix.
    """
    return np.array(
        [
            [0, -vector[2], vector[1]],
            [vector[2], 0, -vector[0]],
            [-vector[1], vector[0], 0],
        ]
    )


def quaternion_super_product(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Calculates the quaternion super product of two quaternions.

    Args:
        q1 (np.ndarray): The first quaternion.
        q2 (np.ndarray): The second quaternion.

    Returns:
        np.ndarray: The quaternion super product.
    """
    eta_0, e_0_x, e_0_y, e_0_z = q1
    eta_1, e_1_x, e_1_y, e_1_z = q2

    e_0 = np.array([e_0_x, e_0_y, e_0_z])
    e_1 = np.array([e_1_x, e_1_y, e_1_z])

    eta_new = eta_0 * eta_1 - (e_0_x * e_1_x + e_0_y * e_1_y + e_0_z * e_1_z)
    nu_new = e_1 * eta_0 + e_0 * eta_1 + np.dot(skew_symmetric(e_0), e_1)

    q_new = quat_norm(np.array([eta_new, nu_new[0], nu_new[1], nu_new[2]]))

    return q_new


def quaternion_error(quat_1: np.ndarray, quat_2: np.ndarray) -> np.ndarray:
    """Calculates the error between two quaternions
    """
    quat_2_inv = np.array([quat_2[0], -quat_2[1], -quat_2[2], -quat_2[3]])

    error_quat = quaternion_super_product(quat_1, quat_2_inv)

    return error_quat


def iterative_quaternion_mean_statequat(
    state_list: list[StateQuat], tol: float = 1e-6, max_iter: int = 100
) -> np.ndarray:
    """Computes the weighted mean of the quaternion orientations from a list of StateQuat objects
    using an iterative approach, without requiring the caller to manually extract the quaternion.

    Parameters:
        state_list (list[StateQuat]): List of StateQuat objects.
        weights (np.ndarray): Weights for each state.
        tol (float): Convergence tolerance.
        max_iter (int): Maximum number of iterations.

    Returns:
        np.ndarray: The averaged quaternion as a 4-element numpy array.
    """
    sigma_quats = [state.orientation for state in state_list]
    n = len(state_list)

    mean_q = sigma_quats[0].copy()

    for _ in range(max_iter):
        weighted_error_vectors = []
        for i, q in enumerate(sigma_quats):
            mean_q_conj = np.array([mean_q[0], -mean_q[1], -mean_q[2], -mean_q[3]])
            e = quaternion_super_product(q, mean_q_conj)

            e0_clipped = np.clip(e[0], -1.0, 1.0)
            angle = 2 * np.arccos(e0_clipped)
            if np.abs(angle) < 1e-8:
                error_vec = np.zeros(3)
            else:
                error_vec = (angle / np.sin(angle / 2)) * e[1:4]
            weighted_error_vectors.append(error_vec)

        error_avg = (1 / n) * np.sum(weighted_error_vectors, axis=0)
        if np.linalg.norm(error_avg) < tol:
            break

        error_norm = np.linalg.norm(error_avg)
        if error_norm > 0:
            delta_q = np.array(
                [
                    np.cos(error_norm / 2),
                    *(np.sin(error_norm / 2) * (error_avg / error_norm)),
                ]
            )
        else:
            delta_q = np.array([1.0, 0.0, 0.0, 0.0])

        mean_q = quaternion_super_product(delta_q, mean_q)
        mean_q = quat_norm(mean_q)

    return mean_q


def mean_set(set_points: list[StateQuat]) -> np.ndarray:
    """Function calculates the mean vector of a set of points

    Args:
        set_points (list[StateQuat]): List of StateQuat objects

    Returns:
        np.ndarray: The mean vector
    """
    n = len(set_points)
    mean_value = StateQuat()

    for state in set_points:
        mean_value.add_without_quaternions(state)

    mean_value = (1 / (n)) * mean_value

    mean_value.orientation = iterative_quaternion_mean_statequat(set_points)

    return mean_value.as_vector()


def mean_measurement(set_points: list[MeasModel]) -> np.ndarray:
    """Function that calculates the mean of a set of points
    """
    n = len(set_points)
    mean_value = MeasModel()

    for state in set_points:
        mean_value = mean_value + state

    mean_value = (1 / n) * mean_value

    return mean_value.measurement


def covariance_set(set_points: list[StateQuat], mean: np.ndarray) -> np.ndarray:
    """Function that calculates the covariance of a set of points
    """
    n = len(set_points)
    covariance = np.zeros(set_points[0].covariance.shape)

    mean_quat = StateQuat()
    mean_quat.fill_states(mean)

    mean_q = mean_quat.orientation

    for state in set_points:
        q = state.orientation
        diff_q = quaternion_error(q, mean_q)

        e0_clipped = np.clip(diff_q[0], -1.0, 1.0)
        angle = 2.0 * np.arccos(e0_clipped)
        if abs(angle) < 1e-8:
            e_vec = np.zeros(3)
        else:
            e_vec = (angle / np.sin(angle / 2)) * diff_q[1:4]

        covariance += np.outer(
            state.subtract(mean_quat, e_vec), state.subtract(mean_quat, e_vec)
        )

    covariance = (1 / (n)) * covariance

    return covariance


def covariance_measurement(set_points: list[MeasModel], mean: np.ndarray) -> np.ndarray:
    """Function that calculates the covariance of a set of points
    """
    n = len(set_points)
    co_size = len(set_points[0].measurement)
    covariance = np.zeros((co_size, co_size))

    mean_meas = MeasModel()
    mean_meas.measurement = mean

    for state in set_points:
        temp_state = state - mean_meas
        covariance += np.outer(temp_state.measurement, temp_state.measurement)

    covariance = (1 / n) * covariance

    return covariance


def cross_covariance(
    set_y: list[StateQuat],
    mean_y: np.ndarray,
    set_z: list[MeasModel],
    mean_z: np.ndarray,
) -> np.ndarray:
    """Calculates the cross covariance between the measurement and state prediction
    """
    n = len(set_y)

    cross_covariance = np.zeros((len(mean_y) - 1, len(mean_z)))
    mean_quat = StateQuat()
    mean_quat.fill_states(mean_y)

    mean_q = mean_quat.orientation

    for i in range(n):
        q = set_y[i].orientation
        diff_q = quaternion_error(q, mean_q)

        e0_clipped = np.clip(diff_q[0], -1.0, 1.0)
        angle = 2.0 * np.arccos(e0_clipped)
        if abs(angle) < 1e-8:
            e_vec = np.zeros(3)
        else:
            e_vec = (angle / np.sin(angle / 2)) * diff_q[1:4]

        cross_covariance += np.outer(
            set_y[i].subtract(mean_quat, e_vec), set_z[i].measurement - mean_z
        )

    cross_covariance = (1 / n) * cross_covariance

    return cross_covariance
