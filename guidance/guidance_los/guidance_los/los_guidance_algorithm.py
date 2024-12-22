#!/usr/bin/env python3

import numpy as np
from rcl_interfaces.msg import ParameterDescriptor, ParameterType


class ThirdOrderLOSGuidance:
    """Third-order Line-of-Sight (LOS) guidance algorithm."""

    def __init__(self):
        # LOS parameters
        self.h_delta_min = 1.0  # Minimum look-ahead distance
        self.h_delta_max = 5.0  # Maximum look-ahead distance
        self.h_delta_factor = 3.0  # Look-ahead scaling factor
        self.nominal_speed = 0.35  # Nominal surge velocity
        self.min_speed = 0.1  # Minimum surge velocity
        self.max_pitch_angle = np.pi / 6  # 30 degrees max pitch
        self.depth_gain = 0.5  # Gain for depth error

        # Filter parameters
        self.x = np.zeros(
            9
        )  # State vector for reference filter [pos, vel, acc] for [surge, pitch, yaw]
        self.omega = np.diag([2.5, 2.5, 2.5])  # Natural frequency - can be tuned
        self.zeta = np.diag([0.7, 0.7, 0.7])  # Damping ratio - can be tuned

        # Initialize filter matrices
        self.setup_filter_matrices()

    def _init_parameters(self):
        """Declare all parameters with default values."""
        # LOS parameters
        self.declare_parameter(
            'los_guidance.h_delta_min',
            1.0,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Minimum look-ahead distance',
            ),
        )

        self.declare_parameter(
            'los_guidance.h_delta_max',
            5.0,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Maximum look-ahead distance',
            ),
        )

        self.declare_parameter(
            'los_guidance.h_delta_factor',
            3.0,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Look-ahead scaling factor',
            ),
        )

        self.declare_parameter(
            'los_guidance.nominal_speed',
            0.35,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Nominal surge velocity',
            ),
        )

        self.declare_parameter(
            'los_guidance.min_speed',
            0.1,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Minimum surge velocity',
            ),
        )

        self.declare_parameter(
            'los_guidance.max_pitch_angle',
            np.pi / 6,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Maximum pitch angle in radians',
            ),
        )

        self.declare_parameter(
            'los_guidance.depth_gain',
            0.5,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE, description='Gain for depth error'
            ),
        )

        # Filter parameters
        self.declare_parameter(
            'los_guidance.filter.omega_diag',
            [2.5, 2.5, 2.5],
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                description='Natural frequency diagonal values',
            ),
        )

        self.declare_parameter(
            'los_guidance.filter.zeta_diag',
            [0.7, 0.7, 0.7],
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                description='Damping ratio diagonal values',
            ),
        )

    def _load_parameters(self):
        """Load parameters from config into class attributes."""
        # LOS parameters
        self.h_delta_min = self.get_parameter('los_guidance.h_delta_min').value
        self.h_delta_max = self.get_parameter('los_guidance.h_delta_max').value
        self.h_delta_factor = self.get_parameter('los_guidance.h_delta_factor').value
        self.nominal_speed = self.get_parameter('los_guidance.nominal_speed').value
        self.min_speed = self.get_parameter('los_guidance.min_speed').value
        self.max_pitch_angle = self.get_parameter('los_guidance.max_pitch_angle').value
        self.depth_gain = self.get_parameter('los_guidance.depth_gain').value

        # Filter parameters
        omega_diag = self.get_parameter('los_guidance.filter.omega_diag').value
        zeta_diag = self.get_parameter('los_guidance.filter.zeta_diag').value
        self.omega = np.diag(omega_diag)
        self.zeta = np.diag(zeta_diag)

    def setup_filter_matrices(self):
        """Setup state-space matrices for the third-order reference filter."""
        self.Ad = np.zeros((9, 9))
        self.Bd = np.zeros((9, 3))

        # Fill Ad matrix blocks
        self.Ad[0:3, 3:6] = np.eye(3)  # Position to velocity coupling
        self.Ad[3:6, 6:9] = np.eye(3)  # Velocity to acceleration coupling

        # Compute characteristic polynomial coefficients
        omega_cubed = self.omega @ self.omega @ self.omega  # ω³
        omega_squared = self.omega @ self.omega  # ω²

        # Fill in the acceleration dynamics
        self.Ad[6:9, 0:3] = -omega_cubed  # -ω³ term
        self.Ad[6:9, 3:6] = (
            -(2 * self.zeta + np.eye(3)) @ omega_squared
        )  # -(2ζ + 1)ω² term
        self.Ad[6:9, 6:9] = -(2 * self.zeta + np.eye(3)) @ self.omega  # -(2ζ + 1)ω term

        # Input matrix
        self.Bd[6:9, :] = omega_cubed  # Acceleration input coupling

    def ssa(self, angle: float) -> float:
        """Maps an angle to the interval [-π, π]."""
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def compute_raw_los_guidance(
        self, current_pos: np.ndarray, target_pos: np.ndarray
    ) -> tuple[np.ndarray, float, float]:
        """Compute raw LOS guidance commands before filtering."""
        # Extract positions
        dx = target_pos[0] - current_pos[0]  # x
        dy = target_pos[1] - current_pos[1]  # y

        # Horizontal plane calculations
        horizontal_distance = np.sqrt(dx**2 + dy**2)
        desired_yaw = np.arctan2(dy, dx)
        yaw_error = self.ssa(desired_yaw - current_pos[3])  # current_pos[3] is yaw

        # Vertical plane calculations
        depth_error = target_pos[2] - current_pos[2]  # z
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
        """Apply third-order reference filter to guidance commands."""
        # Update filter state
        x_dot = self.Ad @ self.x + self.Bd @ commands
        self.x = self.x + x_dot * dt

        # Return filtered position states
        return self.x[0:3]  # Return position states [surge, pitch, yaw]

    def compute_guidance(
        self, current_pos: np.ndarray, target_pos: np.ndarray, dt: float
    ) -> np.ndarray:
        """Compute filtered guidance commands."""
        # Step 1: Compute raw LOS guidance
        (
            raw_commands,
            self.horizontal_distance_to_target,
            self.vertical_distance_to_target,
        ) = self.compute_raw_los_guidance(current_pos, target_pos)

        # Step 2: Apply reference filter
        filtered_commands = self.apply_reference_filter(raw_commands, dt)

        # Normalize yaw angle in filtered commands
        filtered_commands[2] = self.ssa(filtered_commands[2])

        return filtered_commands

    def reset_filter_state(self, current_commands: np.ndarray) -> None:
        """Reset filter state to initial conditions."""
        self.x = np.zeros(9)
        self.x[0:3] = current_commands  # Initialize positions with current commands
        # Initialize velocities and accelerations to zero
        self.x[3:6] = np.zeros(3)  # velocities
        self.x[6:9] = np.zeros(3)  # accelerations
