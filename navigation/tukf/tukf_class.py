import numpy as np
from dataclasses import dataclass, field
from typing import Callable

@dataclass
class AUVState:
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    orientation: np.ndarray = field(default_factory=lambda: np.array(3))
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    angular_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    inertia: np.ndarray = field(default_factory=lambda: np.zeros((9)))
    added_mass: np.ndarray = field(default_factory=lambda: np.zeros((6)))
    damping: np.ndarray = field(default_factory=lambda: np.zeros((6)))
    g_eta: np.ndarray = field(default_factory=lambda: np.zeros((4)))
    covariance: np.ndarray = field(default_factory=lambda: np.zeros((37, 37)))

    def dynamic_part(self) -> np.ndarray:
        """Get the dynamic part of the AUV state."""
        return np.concatenate([
            self.position,
            self.orientation,
            self.velocity,
            self.angular_velocity
        ])
    def okid_part(self) -> np.ndarray:
        """Get the OKID part of the AUV state."""
        return np.concatenate([
            self.inertia,
            self.added_mass,
            self.damping,
            self.g_eta
        ])
    def as_vector(self) -> np.ndarray:
        """Convert the AUV state to a vector representation."""
        return np.concatenate([
            self.position,
            self.orientation,
            self.velocity,
            self.angular_velocity,
            self.inertia.flatten(),
            self.added_mass.flatten(),
            self.damping.flatten(),
            self.g_eta
        ])

    def __add__(self, other: 'AUVState') -> 'AUVState':
        """Add two AUV states together."""
        return AUVState(
            position=self.position + other.position,
            orientation=self.orientation + other.orientation,
            velocity=self.velocity + other.velocity,
            angular_velocity=self.angular_velocity + other.angular_velocity,
            inertia=self.inertia + other.inertia,
            added_mass=self.added_mass + other.added_mass,
            damping=self.damping + other.damping,
            g_eta=self.g_eta + other.g_eta
        )
    def __sub__(self, other: 'AUVState') -> 'AUVState':
        """Subtract two AUV states."""
        return AUVState(
            position=self.position - other.position,
            orientation=self.orientation - other.orientation,
            velocity=self.velocity - other.velocity,
            angular_velocity=self.angular_velocity - other.angular_velocity,
            inertia=self.inertia - other.inertia,
            added_mass=self.added_mass - other.added_mass,
            damping=self.damping - other.damping,
            g_eta=self.g_eta - other.g_eta
        )
    def fill_states(self, x: np.ndarray) -> None:
        """Fill the AUV state with a vector representation."""
        self.position = x[0:3]
        self.orientation = x[3:6]
        self.velocity = x[6:9]
        self.angular_velocity = x[9:12]
        self.inertia = x[12:21]
        self.added_mass = x[21:27]
        self.damping = x[27:33]
        self.g_eta = x[33:37]


@dataclass
class MeasModel:
    """A class defined for a general measurement model."""
    measurement: np.ndarray = field(default_factory=lambda: np.zeros(3))
    covariance: np.ndarray = field(default_factory=lambda: np.zeros((3, 3)))
    H: Callable[["AUVState"], "MeasModel"] | None = None

    def __post_init__(self):
        """Initialize H with a default measurement function if none provided."""
        if self.H is None:
            self.H = self._default_H
    
    def _default_H(self, state: AUVState) -> 'MeasModel':
        """Default measurement function that returns velocity."""
        H_matrix = np.zeros((3, 12))
        H_matrix[:, 6:9] = np.eye(3)
        z_i = MeasModel()
        z_i.measurement = np.dot(H_matrix, state.dynamic_part())
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

def generate_delta_matrix_2(n: float) -> np.ndarray:
    """Generates the weight matrix used in the TUKF sigma point generation.

    Parameters:
        n (int): The state dimension.

    Returns:
        delta (np.ndarray): An n x 2n orthonormal transformation matrix used to generate TUKF sigma points.
    """
    delta = np.zeros((n, 2 * n))
    k = 0.001  # Tuning parameter to ensure pos def

    for i in range(2 * n):
        for j in range(n // 2):
            delta[2 * j + 1, i] = (
                np.sqrt(2) * np.sin(2 * j - 1) * ((k * np.pi) / n)
            )
            delta[2 * j, i] = np.sqrt(2) * np.cos(2 * j - 1) * ((k * np.pi) / n)

        if (n % 2) == 1:
            delta[n - 1, i] = np.sqrt(2) * np.cos(2 * j - 1) * ((k * np.pi) / n)

    return delta

def generate_delta_matrix(n: int) -> np.ndarray:
    if n < 1:
        raise ValueError("n must be a positive integer")

    delta = np.zeros((n, 2 * n))
    r_max = n // 2                  # floor(n/2)
    sq2 = np.sqrt(2.0)

    for k in range(1, 2 * n + 1):   # k = 1 … 2n
        for r in range(1, r_max + 1):
            row_cos = 2 * r - 2     # 0‑based index for γ_{k,2r‑1}
            row_sin = 2 * r - 1     # 0‑based index for γ_{k,2r}
            angle = (2 * r - 1) * k * np.pi / n
            delta[row_cos, k - 1] = sq2 * np.cos(angle)
            delta[row_sin, k - 1] = sq2 * np.sin(angle)

        if n % 2 == 1:              # extra entry when n is odd
            delta[n - 1, k - 1] = (-1) ** k

    return delta

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

def mean_set(set_points: list[AUVState]) -> np.ndarray:
    """Function calculates the mean vector of a set of points.

    Args:
        set_points (list[AUVState]): List of AUVState objects

    Returns:
        np.ndarray: The mean vector
    """
    n = len(set_points)
    mean_value = np.zeros(set_points[0].as_vector().shape)

    for state in set_points:
        mean_value = mean_value + state.as_vector()

    mean_value = (1 / n) * mean_value

    return mean_value


def mean_measurement(set_points: list[MeasModel]) -> np.ndarray:
    """Function that calculates the mean of a set of points."""
    n = len(set_points)
    mean_value = MeasModel()

    for state in set_points:
        mean_value = mean_value + state

    mean_value = (1 / n) * mean_value

    return mean_value.measurement


def covariance_set(set_points: list[AUVState], mean: np.ndarray) -> np.ndarray:
    """Function that calculates the covariance of a set of points."""
    n = len(set_points)
    covariance = np.zeros(set_points[0].covariance.shape)

    for state in set_points:
        W_i = state.as_vector() - mean

        covariance += np.outer(W_i, W_i)

    covariance = (1 / n) * covariance

    return covariance


def covariance_measurement(set_points: list[MeasModel], mean: np.ndarray) -> np.ndarray:
    """Function that calculates the covariance of a set of points."""
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
    set_y: list[AUVState],
    mean_y: np.ndarray,
    set_z: list[MeasModel],
    mean_z: np.ndarray,
) -> np.ndarray:
    """Calculates the cross covariance between the measurement and state prediction."""
    n = len(set_y)

    cross_covariance = np.zeros((len(mean_y), len(mean_z)))

    for i in range(n):
        state_diff = set_y[i].as_vector() - mean_y
        meas_diff = set_z[i].measurement - mean_z

        cross_covariance += np.outer(state_diff, meas_diff)
    
    cross_covariance = (1 / n) * cross_covariance

    return cross_covariance


# -----------------------------------------------------------

def rotation_matrix(euler_angles: np.ndarray) -> np.ndarray:
    """Calculates the rotation matrix from Euler angles (roll, pitch, yaw)."""
    roll, pitch, yaw = euler_angles
    
    # Roll rotation
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    
    # Pitch rotation
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    
    # Yaw rotation
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    
    # Complete rotation matrix
    R = Rz @ Ry @ Rx
    return R

def angular_velocity_transformation(euler_angles: np.ndarray) -> np.ndarray:
    """Transformation matrix relating Euler rates to angular velocities."""
    roll, pitch, yaw = euler_angles
    
    T = np.array([
        [1, 0, -np.sin(pitch)],
        [0, np.cos(roll), np.cos(pitch) * np.sin(roll)],
        [0, -np.sin(roll), np.cos(pitch) * np.cos(roll)]
    ])
    
    return T

def M_rb(inertia: np.ndarray) -> np.ndarray:
    m = 25.5
    inertia = inertia.reshape((3, 3))
    r_b_bg = np.array([0.01, 0.0, 0.02])
    M_rb = np.zeros((6, 6))
    M_rb[0:3, 0:3] = m * np.eye(3)
    M_rb[3:6, 3:6] = inertia
    M_rb[0:3, 3:6] = -m * skew_symmetric(r_b_bg)
    M_rb[3:6, 0:3] = m * skew_symmetric(r_b_bg)
    return M_rb

def M_a(added_mass: np.ndarray) -> np.ndarray:
    """Calculates the added mass matrix."""
    M_a = np.zeros((6, 6))
    M_a[0:3, 0:3] = np.diag(added_mass[0:3])
    M_a[3:6, 3:6] = np.diag(added_mass[3:6])
    return M_a

def C_rb(inertia: np.ndarray, angular_velocity: np.ndarray) -> np.ndarray:
    """Calculates the Coriolis matrix."""
    m = 25.5
    r_b_bg = np.array([0.01, 0.0, 0.02])
    inertia = inertia.reshape((3, 3))
    C_rb = np.zeros((6, 6))

    C_rb[0:3, 0:3] = m * skew_symmetric(angular_velocity)
    C_rb[3:6, 3:6] = -skew_symmetric(np.dot(inertia, angular_velocity))
    C_rb[0:3, 3:6] = -m * skew_symmetric(angular_velocity) @ skew_symmetric(r_b_bg)
    C_rb[3:6, 0:3] = m * skew_symmetric(r_b_bg) @ skew_symmetric(angular_velocity)
    return C_rb

def C_a(added_mass: np.ndarray, angular_velocity: np.ndarray, velocity: np.ndarray) -> np.ndarray:
    """Calculates the added mass Coriolis matrix."""
    C_a = np.zeros((6, 6))
    A11 = np.diag(added_mass[0:3])
    A22 = np.diag(added_mass[3:6])
    C_a[3:6,3:6] = - skew_symmetric(A22 @ angular_velocity)
    C_a[0:3,3:6] = - skew_symmetric(A11 @ velocity)
    C_a[3:6,0:3] = - skew_symmetric(A11 @ velocity)
    return C_a

def D_linear(damping_linear: np.ndarray) -> np.ndarray:
    """Calculates the linear damping matrix."""
    D = np.zeros((6, 6))
    D[0:3, 0:3] = -np.diag(damping_linear[0:3])
    D[3:6, 3:6] = -np.diag(damping_linear[3:6])
    return D

def g_eta(g_eta: np.ndarray, orientation: np.ndarray) -> np.ndarray:
    """Calculates the g_eta matrix using Euler angles."""
    Delta_WB = g_eta[0]
    M_x = g_eta[1]
    M_y = g_eta[2]
    M_z = g_eta[3]

    # Get rotation matrix using Euler angles
    R = rotation_matrix(orientation)
    
    G_eta = np.zeros((6,1))
    # Gravitational forces
    G_eta[0:3] = -Delta_WB * R[:, 2].reshape(3, 1)
    
    # Buoyancy moments
    G_eta[3] = -M_y * R[2, 2] + M_z * R[1, 2]
    G_eta[4] = -M_z * R[0, 2] + M_x * R[2, 2]
    G_eta[5] = -M_x * R[1, 2] + M_y * R[0, 2]

    return G_eta

def F_dynamics(
    state: AUVState,
    dt: float,
    control_input: np.ndarray) -> AUVState:

    """Calculates the dynamics of the system."""
    m_rb = M_rb(state.inertia)
    m_a = M_a(state.added_mass)
    c_rb = C_rb(state.inertia, state.angular_velocity)
    c_a = C_a(state.added_mass, state.angular_velocity, state.velocity)
    D_l = D_linear(state.damping)
    g_eta_ = g_eta(state.g_eta, state.orientation)
    
    # Get rotation and transformation matrices
    r = rotation_matrix(state.orientation)
    t = angular_velocity_transformation(state.orientation)
    
    Crb = c_rb + c_a
    Mrb = m_rb + m_a
    M_inv = np.linalg.inv(Mrb)
    
    # Create a vector of velocity and angular velocity
    nu = np.concatenate([state.velocity, state.angular_velocity])

    # Calculate the new state
    state_dot = AUVState()
    state_dot.position = np.dot(r, state.velocity)
    
    # Calculate Euler angle rates from angular velocities
    t_inv = np.linalg.inv(t)
    euler_rates = np.dot(t_inv, state.angular_velocity)
    state_dot.orientation = euler_rates
    
    Nu = M_inv @ (control_input - np.dot(Crb, nu) - np.dot(D_l, nu) - g_eta_.flatten())
    
    state_dot.velocity = Nu[:3]
    state_dot.angular_velocity = Nu[3:6]

    # Update the inertia, added mass, damping, and g_eta
    state_dot.inertia = np.zeros_like(state.inertia)
    state_dot.added_mass = np.zeros_like(state.added_mass)
    state_dot.damping = np.zeros_like(state.damping)
    state_dot.g_eta = np.zeros_like(state.g_eta)

    new_state = AUVState()
    new_state.position = state.position + state_dot.position * dt
    new_state.orientation = state.orientation + state_dot.orientation * dt
    new_state.velocity = state.velocity + state_dot.velocity * dt
    new_state.angular_velocity = state.angular_velocity + state_dot.angular_velocity * dt
    new_state.inertia = state.inertia + state_dot.inertia * dt
    new_state.added_mass = state.added_mass + state_dot.added_mass * dt
    new_state.damping = state.damping + state_dot.damping * dt
    new_state.g_eta = state.g_eta + state_dot.g_eta * dt

    return new_state
# -----------------------------------------------------------
