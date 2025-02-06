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
    lookahead_distance_max: float = 8.0
    lookahead_distance_factor: float = 1.0
    nominal_speed: float = 1.0
    min_speed: float = 0.1
    max_pitch_angle: float = 0.5  # ~28.6 degrees
    depth_gain: float = 1.0
    dt: float = 0.01


@dataclass(slots=True)
class FilterParameters:
    """Parameters for third-order filter.

    Attributes:
        omega_diag: Natural frequencies for surge, pitch, and yaw [rad/s]
        zeta_diag: Damping ratios for surge, pitch, and yaw [-]
    """

    omega_diag: np.ndarray = field(default_factory=lambda: np.array([1.0, 1.0, 1.0]))
    zeta_diag: np.ndarray = field(default_factory=lambda: np.array([1.0, 1.0, 1.0]))


class LOSGuidance:
    """This class implements the Line-of-Sight (LOS) guidance algorithm.

    The LOS provide a control outputs for surge, pitch, and yaw for navigating towards a target in 3D space.
    """

    def __init__(self, los_params: LOSParameters, filter_params: FilterParameters):
        self.los_params = los_params
        self.filter_params = filter_params
        self.x = np.zeros(9)  # Filter state
        self.horizontal_distance = 0.0
        self.setup_filter_matrices()
        self.lookahead_h = 2.0
        self.lookahead_v = 2.0

    def setup_filter_matrices(self):
        omega = np.diag(self.filter_params.omega_diag)
        zeta = np.diag(self.filter_params.zeta_diag)
        omega_cubed = omega @ omega @ omega
        omega_squared = omega @ omega

        self.Ad = np.zeros((9, 9))
        self.Bd = np.zeros((9, 3))

        self.Ad[0:3, 3:6] = np.eye(3)
        self.Ad[3:6, 6:9] = np.eye(3)
        self.Ad[6:9, 0:3] = -omega_cubed
        self.Ad[6:9, 3:6] = -(2 * zeta + np.eye(3)) @ omega_squared
        self.Ad[6:9, 6:9] = -(2 * zeta + np.eye(3)) @ omega

        self.Bd[6:9, :] = omega_cubed

    def compute_raw_los_guidance(self, current_pos: State, target_pos: State) -> State:
        dx = target_pos.pose.x - current_pos.pose.x
        dy = target_pos.pose.y - current_pos.pose.y

        self.horizontal_distance = np.sqrt(dx**2 + dy**2)
        desired_yaw = np.arctan2(dy, dx)
        yaw_error = ssa(desired_yaw - current_pos.pose.yaw)

        depth_error = target_pos.pose.z - current_pos.pose.z
        desired_pitch = self.compute_pitch_command(
            depth_error, self.horizontal_distance
        )

        desired_surge = self.compute_desired_speed(yaw_error, self.horizontal_distance)
        commands = State()
        commands.twist.linear_x = desired_surge
        commands.pose.pitch = desired_pitch
        commands.pose.yaw = desired_yaw

        return commands

    def compute_pitch_command(
        self, depth_error: float, horizontal_distance: float
    ) -> float:
        """Compute pitch command with distance-based scaling."""
        raw_pitch = -self.los_params.depth_gain * depth_error
        distance_factor = min(
            1.0, horizontal_distance / self.los_params.lookahead_distance_max
        )
        modified_pitch = raw_pitch * distance_factor
        return np.clip(
            modified_pitch,
            -self.los_params.max_pitch_angle,
            self.los_params.max_pitch_angle,
        )

    def compute_desired_speed(
        self, yaw_error: float, distance_to_target: float
    ) -> float:
        """Compute speed command with yaw and distance-based scaling."""
        yaw_factor = np.cos(yaw_error)
        distance_factor = min(
            1.0, distance_to_target / self.los_params.lookahead_distance_max
        )
        desired_speed = self.los_params.nominal_speed * yaw_factor * distance_factor
        return max(self.los_params.min_speed, desired_speed)

    def apply_reference_filter(self, commands: State) -> State:
        los_array = self.state_to_los_array(commands)
        x_dot = self.Ad @ self.x + self.Bd @ los_array
        self.x = self.x + x_dot * self.los_params.dt

        commands.twist.linear_x = self.x[0]
        commands.pose.pitch = self.x[1]
        commands.pose.yaw = self.x[2]

        return commands

    def compute_guidance(self, current_pos: State, target_pos: State) -> State:
        raw_commands = self.compute_raw_los_guidance(current_pos, target_pos)

        filtered_commands = self.apply_reference_filter(raw_commands)
        filtered_commands.pose.pitch = ssa(filtered_commands.pose.pitch)
        filtered_commands.pose.yaw = ssa(filtered_commands.pose.yaw)
        return filtered_commands

    def reset_filter_state(self, current_state: State) -> None:
        self.x = np.zeros(9)
        current_state_array = self.state_to_los_array(current_state)
        self.x[0:3] = current_state_array

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
