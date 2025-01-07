from dataclasses import dataclass, field

import numpy as np


@dataclass
class State:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    surge_vel: float = 0.0

    def __add__(self, other: "State") -> "State":
        return State(
            x=self.x + other.x,
            y=self.y + other.y,
            z=self.z + other.z,
            pitch=self.pitch + other.pitch,
            yaw=self.yaw + other.yaw,
        )

    def __sub__(self, other: "State") -> "State":
        return State(
            x=self.x - other.x,
            y=self.y - other.y,
            z=self.z - other.z,
            pitch=self.pitch - other.pitch,
            yaw=self.yaw - other.yaw,
        )

    def as_los_array(self):
        return np.array([self.surge_vel, self.pitch, self.yaw])


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


class ThirdOrderLOSGuidance:
    """This class implements the Line-of-Sight (LOS) guidance algorithm.

    The LOS provide a control outputs for surge, pitch, and yaw for navigating towards a target in 3D space.
    """

    def __init__(self, los_params: LOSParameters, filter_params: FilterParameters):
        self.los_params = los_params
        self.filter_params = filter_params
        self.x = np.zeros(9)  # Filter state
        self.horizontal_distance = 0.0
        self.setup_filter_matrices()

    @staticmethod
    def ssa(angle: float) -> float:
        return (angle + np.pi) % (2 * np.pi) - np.pi

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
        dx = target_pos.x - current_pos.x
        dy = target_pos.y - current_pos.y

        self.horizontal_distance = np.sqrt(dx**2 + dy**2)
        desired_yaw = np.arctan2(dy, dx)
        yaw_error = self.ssa(desired_yaw - current_pos.yaw)

        depth_error = target_pos.z - current_pos.z
        desired_pitch = self.compute_pitch_command(
            depth_error, self.horizontal_distance
        )

        desired_surge = self.compute_desired_speed(yaw_error, self.horizontal_distance)

        commands = State(surge_vel=desired_surge, pitch=desired_pitch, yaw=desired_yaw)

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
        x_dot = self.Ad @ self.x + self.Bd @ commands.as_los_array()
        self.x = self.x + x_dot * self.los_params.dt
        return State(surge_vel=self.x[0], pitch=self.x[1], yaw=self.x[2])

    def compute_guidance(self, current_pos: State, target_pos: State) -> State:
        raw_commands = self.compute_raw_los_guidance(current_pos, target_pos)

        filtered_commands = self.apply_reference_filter(raw_commands)
        filtered_commands.pitch = self.ssa(filtered_commands.pitch)
        filtered_commands.yaw = self.ssa(filtered_commands.yaw)
        return filtered_commands

    def reset_filter_state(self, current_state: State) -> None:
        self.x = np.zeros(9)
        self.x[0:3] = current_state.as_los_array()