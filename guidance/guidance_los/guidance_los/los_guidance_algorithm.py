from dataclasses import dataclass, field

import numpy as np
from vortex_utils.python_utils import State, ssa


@dataclass(slots=True)
class LOSParameters:
    """Parameters for Line-of-Sight guidance algorithm.

    Attributes:
        lookahead_distance_min: Minimum lookahead distance in meters
        lookahead_distance_max: Maximum lookahead distance in meters
        lookahead_distance_factor: Factor to adjust lookahead distance based on cross-track error
        nominal_speed: Desired cruising speed in m/s
        min_speed: Minimum allowed speed in m/s
        max_pitch_angle: Maximum allowed pitch angle in radians
        depth_gain: Gain for depth control
        dt: update rate in seconds
    """

    lookahead_distance_min: float = 2.0
    lookahead_distance_max: float = 2.0
    lookahead_distance_factor: float = 1.0
    nominal_speed: float = 1.0
    min_speed: float = 0.1
    max_pitch_angle: float = 0.5  # ~28.6 degrees
    depth_gain: float = 1.0
    dt: float = 0.01


@dataclass(slots=True)
class FilterParameters:
    """Parameters for filter.

    Attributes:
        omega_diag: Natural frequencies for surge, pitch, and yaw [rad/s]
        zeta_diag: Damping ratios for surge, pitch, and yaw [-]
    """

    omega_diag: np.ndarray = field(default_factory=lambda: np.array([1.0, 1.0, 1.0]))
    zeta_diag: np.ndarray = field(default_factory=lambda: np.array([1.0, 1.0, 1.0]))


class LOSGuidanceAlgorithm:
    """This class implements the Line-of-Sight (LOS) guidance algorithm.

    The LOS provide a control outputs for surge, pitch, and yaw for navigating towards a target in 3D space.
    """

    def __init__(self, los_params: LOSParameters, filter_params: FilterParameters):
        self.los_params = los_params
        self.filter_params = filter_params
        self.filter_state = np.zeros(9)
        self.horizontal_distance = 0.0
        self.setup_filter_matrices()
        self.lookahead_h = 2.0
        self.lookahead_v = 2.0

    def setup_filter_matrices(self):
        omega = np.diag(self.filter_params.omega_diag)
        zeta = np.diag(self.filter_params.zeta_diag)
        omega_cubed = np.linalg.matrix_power(omega, 3)
        omega_squared = np.linalg.matrix_power(omega, 2)

        self.Ad = np.zeros((9, 9))
        self.Bd = np.zeros((9, 3))

        self.Ad[0:3, 3:6] = np.eye(3)
        self.Ad[3:6, 6:9] = np.eye(3)
        self.Ad[6:9, 0:3] = -omega_cubed
        self.Ad[6:9, 3:6] = -np.dot((2 * zeta + np.eye(3)), omega_squared)
        self.Ad[6:9, 6:9] = -np.dot((2 * zeta + np.eye(3)), omega)

        self.Bd[6:9, :] = omega_cubed

    def compute_raw_los_guidance(self, current_pos: State, target_pos: State) -> State:
        dx = target_pos.pose.x - current_pos.pose.x
        dy = target_pos.pose.y - current_pos.pose.y

        self.horizontal_distance = np.sqrt(dx**2 + dy**2)
        desired_yaw = np.arctan2(dy, dx)
        yaw_error = ssa(desired_yaw - current_pos.pose.yaw)

        alpha_c = self.calculate_alpha_c(
            current_pos.twist.linear_x,
            current_pos.twist.linear_y,
            current_pos.twist.linear_z,
            current_pos.pose.roll,
        )

        _, _, crosstrack_z = self.compute_crosstrack_error(
            current_pos, current_pos, target_pos
        )

        desired_pitch = self.compute_pitch_command(
            current_pos, target_pos, crosstrack_z, alpha_c
        )

        desired_surge = self.compute_desired_speed(
            current_pos, target_pos, abs(crosstrack_z)
        )

        commands = State()
        commands.twist.linear_x = desired_surge
        commands.pose.pitch = desired_pitch
        commands.pose.yaw = desired_yaw

        return commands

    def compute_pitch_command(
        self,
        current_waypoint: State,
        next_waypoint: State,
        crosstrack_z: float,
        alpha_c: float,
    ) -> float:
        """Compute pitch command based on LOS guidance law."""
        pi_v = self.compute_pi_v(current_waypoint, next_waypoint)

        theta_d = pi_v + alpha_c + np.arctan(crosstrack_z / self.lookahead_v)

        theta_d = ssa(theta_d)
        return np.clip(
            theta_d, -self.los_params.max_pitch_angle, self.los_params.max_pitch_angle
        )

    def compute_desired_speed(
        self, current_pos: State, target_pos: State, crosstrack_error: float
    ) -> float:
        """Compute speed command based on path-following requirements."""
        dx = target_pos.pose.x - current_pos.pose.x
        dy = target_pos.pose.y - current_pos.pose.y
        dz = target_pos.pose.z - current_pos.pose.z
        distance_to_target = np.sqrt(dx**2 + dy**2 + dz**2)

        crosstrack_factor = np.exp(
            -self.los_params.lookahead_distance_factor * abs(crosstrack_error)
        )

        distance_factor = min(
            1.0, distance_to_target / self.los_params.lookahead_distance_max
        )

        desired_speed = (
            self.los_params.nominal_speed * crosstrack_factor * distance_factor
        )

        return max(self.los_params.min_speed, desired_speed)

    def compute_crosstrack_error(
        self, current_pos: State, current_waypoint: State, next_waypoint: State
    ) -> tuple[float, float, float]:
        """Compute cross-track errors in path-tangential frame."""
        path_vector = self.state_as_pos_array(next_waypoint - current_waypoint)
        path_length = np.linalg.norm(path_vector)
        if path_length == 0:
            return 0.0, 0.0, 0.0

        pi_h = self.compute_pi_h(current_waypoint, next_waypoint)
        pi_v = self.compute_pi_v(current_waypoint, next_waypoint)

        R_y = self.compute_rotation_y(pi_v)
        R_z = self.compute_rotation_z(pi_h)
        R = R_y.T @ R_z.T

        pos_error = self.state_as_pos_array(current_pos - current_waypoint)
        error_path = R @ pos_error
        along_track = error_path[0]

        cross_track_y = error_path[1]
        cross_track_z = error_path[2]

        return along_track, cross_track_y, cross_track_z

    def apply_reference_filter(self, commands: State) -> State:
        los_array = self.state_to_los_array(commands)
        x_dot = self.Ad @ self.filter_state + self.Bd @ los_array
        self.filter_state = self.filter_state + x_dot * self.los_params.dt

        commands.twist.linear_x = self.filter_state[0]
        commands.pose.pitch = self.filter_state[1]
        commands.pose.yaw = self.filter_state[2]

        return commands

    def compute_guidance(self, current_pos: State, target_pos: State) -> State:
        raw_commands = self.compute_raw_los_guidance(current_pos, target_pos)

        filtered_commands = self.apply_reference_filter(raw_commands)
        filtered_commands.pose.pitch = ssa(filtered_commands.pose.pitch)
        filtered_commands.pose.yaw = ssa(filtered_commands.pose.yaw)
        return filtered_commands

    def reset_filter_state(self, current_state: State) -> None:
        self.filter_state = np.zeros(9)
        current_state_array = self.state_to_los_array(current_state)
        self.filter_state[0:3] = current_state_array

    @staticmethod
    def state_to_los_array(state: State) -> np.ndarray:
        """Converts State object to array with surge velocity, pitch, and yaw."""
        return np.array([state.twist.linear_x, state.pose.pitch, state.pose.yaw])

    @staticmethod
    def state_as_pos_array(state: State) -> np.ndarray:
        """Converts State object to array with x, y, z."""
        return np.array([state.pose.x, state.pose.y, state.pose.z])

    @staticmethod
    def compute_pi_h(current_waypoint: State, next_waypoint: State) -> float:
        dx = next_waypoint.pose.x - current_waypoint.pose.x
        dy = next_waypoint.pose.y - current_waypoint.pose.y
        return np.arctan2(dy, dx)

    @staticmethod
    def compute_pi_v(current_waypoint: State, next_waypoint: State) -> float:
        dz = next_waypoint.pose.z - current_waypoint.pose.z
        horizontal_distance = np.sqrt(
            (next_waypoint.pose.x - current_waypoint.pose.x) ** 2
            + (next_waypoint.pose.y - current_waypoint.pose.y) ** 2
        )
        return np.arctan2(-dz, horizontal_distance)

    @staticmethod
    def compute_rotation_y(pi_v: float) -> np.ndarray:
        return np.array(
            [
                [np.cos(pi_v), 0, np.sin(pi_v)],
                [0, 1, 0],
                [-np.sin(pi_v), 0, np.cos(pi_v)],
            ]
        )

    @staticmethod
    def compute_rotation_z(pi_h: float) -> np.ndarray:
        return np.array(
            [
                [np.cos(pi_h), -np.sin(pi_h), 0],
                [np.sin(pi_h), np.cos(pi_h), 0],
                [0, 0, 1],
            ]
        )

    def compute_psi_d(
        self,
        current_waypoint: State,
        next_waypoint: State,
        crosstrack_y: float,
        beta_c: float,
    ) -> float:
        pi_h = self.compute_pi_h(current_waypoint, next_waypoint)
        psi_d = pi_h - np.arctan(crosstrack_y / self.lookahead_h) - beta_c
        psi_d = ssa(psi_d)
        return psi_d

    def compute_theta_d(
        self,
        current_waypoint: State,
        next_waypoint: State,
        crosstrack_z: float,
        alpha_c: float,
    ) -> float:
        pi_v = self.compute_pi_v(current_waypoint, next_waypoint)
        theta_d = pi_v + np.arctan(crosstrack_z / self.lookahead_v) + alpha_c
        theta_d = ssa(theta_d)
        return theta_d

    @staticmethod
    def calculate_alpha_c(u: float, v: float, w: float, phi: float) -> float:
        if u == 0:
            return 0
        return np.arctan(
            (v * np.sin(phi) + w * np.cos(phi)) / u
        )  # Slide 104 in Fossen 2024

    @staticmethod
    def calculate_beta_c(
        u: float, v: float, w: float, phi: float, theta: float, alpha_c: float
    ) -> float:
        u_v = u * np.sqrt(1 + np.tan(alpha_c) ** 2)
        if u_v == 0:
            return 0
        return np.arctan(
            (v * np.cos(phi) - w * np.sin(phi)) / (u_v * np.cos(theta - alpha_c))
        )  # Slide 104 in Fossen 2024
