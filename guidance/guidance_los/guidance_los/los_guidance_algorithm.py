import numpy as np


class ThirdOrderLOSGuidance:
    """This class implements the Line-of-Sight (LOS) guidance algorithm.

    The LOS provide a control outputs for surge, pitch, and yaw for navigating towards a target in 3D space.
    """

    def __init__(self, config_dict: dict):
        self.config_dict = config_dict
        self.init_los_parameters()
        self.init_filter_parameters()
        self.setup_filter_matrices()

    def init_los_parameters(self):
        """Initialize Line-of-Sight guidance parameters."""
        self.h_delta_min = self.config_dict['h_delta_min']
        self.h_delta_max = self.config_dict['h_delta_max']
        self.h_delta_factor = self.config_dict['h_delta_factor']
        self.nominal_speed = self.config_dict['nominal_speed']
        self.min_speed = self.config_dict['min_speed']
        self.max_pitch_angle = self.config_dict['max_pitch_angle']
        self.depth_gain = self.config_dict['depth_gain']

    def init_filter_parameters(self):
        """Initialize filter state and parameters."""
        self.x = np.zeros(9)
        self.omega = np.diag(self.config_dict['filter']['omega_diag'])
        self.zeta = np.diag(self.config_dict['filter']['zeta_diag'])

    @staticmethod
    def ssa(angle: float) -> float:
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def setup_filter_matrices(self):
        self.Ad = np.zeros((9, 9))
        self.Bd = np.zeros((9, 3))

        # Fill Ad matrix blocks
        self.Ad[0:3, 3:6] = np.eye(3)
        self.Ad[3:6, 6:9] = np.eye(3)

        # Compute characteristic polynomial coefficients
        omega_cubed = self.omega @ self.omega @ self.omega
        omega_squared = self.omega @ self.omega

        # Fill in the acceleration dynamics
        self.Ad[6:9, 0:3] = -omega_cubed
        self.Ad[6:9, 3:6] = -(2 * self.zeta + np.eye(3)) @ omega_squared
        self.Ad[6:9, 6:9] = -(2 * self.zeta + np.eye(3)) @ self.omega

        # Input matrix
        self.Bd[6:9, :] = omega_cubed

    def compute_raw_los_guidance(
        self, current_pos: np.ndarray, target_pos: np.ndarray
    ) -> tuple[np.ndarray, float, float]:
        # Extract positions
        dx = target_pos[0] - current_pos[0]
        dy = target_pos[1] - current_pos[1]

        # Horizontal plane calculations
        horizontal_distance = np.sqrt(dx**2 + dy**2)
        desired_yaw = np.arctan2(dy, dx)
        yaw_error = self.ssa(desired_yaw - current_pos[3])

        # Vertical plane calculations
        depth_error = target_pos[2] - current_pos[2]
        desired_pitch = self.compute_pitch_command(depth_error, horizontal_distance)

        # Compute desired surge velocity
        desired_surge = self.compute_desired_speed(yaw_error, horizontal_distance)

        # Return commands as numpy array
        commands = np.array([desired_surge, desired_pitch, desired_yaw])
        return commands, horizontal_distance, depth_error

    def compute_pitch_command(
        self, depth_error: float, horizontal_distance: float
    ) -> float:
        """Compute pitch command with distance-based scaling."""
        raw_pitch = -self.depth_gain * depth_error
        distance_factor = min(1.0, horizontal_distance / self.h_delta_max)
        modified_pitch = raw_pitch * distance_factor
        return np.clip(modified_pitch, -self.max_pitch_angle, self.max_pitch_angle)

    def compute_desired_speed(self, yaw_error: float, distance: float) -> float:
        """Compute speed command with yaw and distance-based scaling."""
        yaw_factor = np.cos(yaw_error)
        distance_factor = min(1.0, distance / self.h_delta_max)
        desired_speed = self.nominal_speed * yaw_factor * distance_factor
        return max(self.min_speed, desired_speed)

    def apply_reference_filter(self, commands: np.ndarray, dt: float) -> np.ndarray:
        # Update filter state
        x_dot = self.Ad @ self.x + self.Bd @ commands
        self.x = self.x + x_dot * dt
        return self.x[0:3]  # Return position states [surge, pitch, yaw]

    def compute_guidance(
        self, current_pos: np.ndarray, target_pos: np.ndarray, dt: float
    ) -> np.ndarray:
        # Compute raw LOS guidance
        (
            raw_commands,
            self.horizontal_distance_to_target,
            self.vertical_distance_to_target,
        ) = self.compute_raw_los_guidance(current_pos, target_pos)

        # Apply reference filter
        filtered_commands = self.apply_reference_filter(raw_commands, dt)
        filtered_commands[2] = self.ssa(filtered_commands[2])
        return filtered_commands

    def reset_filter_state(self, current_commands: np.ndarray) -> None:
        self.x = np.zeros(9)
        self.x[0:3] = current_commands
        self.x[3:6] = np.zeros(3)
        self.x[6:9] = np.zeros(3)
