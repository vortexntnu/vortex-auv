#!/usr/bin/env python3
"""Third-order Line-of-Sight (LOS) guidance algorithm with reference filtering.

This module implements a Line-of-Sight guidance algorithm with third-order reference
filtering for smooth motion control. The algorithm generates guidance commands
(surge, pitch, yaw) based on current position and target position while ensuring
smooth transitions through third-order filtering.

Key features:
- LOS guidance with look-ahead distance scaling
- Third-order reference filtering for smooth motion
- Adaptive speed control based on cross-track error
- Configurable natural frequency and damping parameters
"""

from typing import Tuple

import numpy as np


class ThirdOrderLOSGuidance:
    """Third-order Line-of-Sight guidance algorithm implementation.

    This class provides methods for computing guidance commands using LOS algorithm
    with third-order reference filtering. It handles both horizontal and vertical
    plane guidance while ensuring smooth transitions through filtering.

    Attributes:
        h_delta_min (float): Minimum look-ahead distance for LOS calculation
        h_delta_max (float): Maximum look-ahead distance for LOS calculation
        h_delta_factor (float): Scaling factor for look-ahead distance
        nominal_speed (float): Nominal surge velocity for forward motion
        min_speed (float): Minimum allowable surge velocity
        max_pitch_angle (float): Maximum allowable pitch angle (±30 degrees)
        depth_gain (float): Proportional gain for depth control
        x (np.ndarray): State vector for reference filter [pos, vel, acc] for each channel
        omega (np.ndarray): Natural frequency diagonal matrix for filter tuning
        zeta (np.ndarray): Damping ratio diagonal matrix for filter tuning
    """

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

    def setup_filter_matrices(self):
        """Setup state-space matrices for the third-order reference filter.

        Creates the state transition matrix (Ad) and input matrix (Bd) for the
        third-order filter based on natural frequency (omega) and damping ratio (zeta).
        The resulting system provides smooth transitions in position, velocity,
        and acceleration for each control channel.
        """
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
        """Maps an angle to the interval [-π, π].

        Args:
            angle (float): Input angle in radians

        Returns:
            float: Normalized angle in range [-π, π]
        """
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def compute_raw_los_guidance(
        self, current_pos: np.ndarray, target_pos: np.ndarray
    ) -> Tuple[np.ndarray, float, float]:
        """Compute raw LOS guidance commands before filtering.

        Calculates desired surge velocity, pitch angle, and yaw angle based on
        current position and target position using LOS guidance principles.

        Args:
            current_pos (np.ndarray): Current position and orientation [x, y, z, yaw, pitch]
            target_pos (np.ndarray): Target position [x, y, z]

        Returns:
            Tuple containing:
            - np.ndarray: Raw commands [surge, pitch, yaw]
            - float: Horizontal distance to target
            - float: Depth error
        """
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
        """Compute pitch command with distance-based scaling.

        Calculates desired pitch angle based on depth error and horizontal distance,
        with scaling based on distance to prevent steep angles when close to target.

        Args:
            depth_error (float): Error in depth (desired - current)
            horizontal_distance (float): Distance to target in horizontal plane

        Returns:
            float: Commanded pitch angle in radians, limited to ±max_pitch_angle
        """
        raw_pitch = -self.depth_gain * depth_error
        distance_factor = min(1.0, horizontal_distance / self.h_delta_max)
        modified_pitch = raw_pitch * distance_factor
        return np.clip(modified_pitch, -self.max_pitch_angle, self.max_pitch_angle)

    def compute_desired_speed(self, yaw_error: float, distance: float) -> float:
        """Compute speed command with yaw and distance-based scaling.

        Reduces speed when large yaw corrections are needed or when close to target.

        Args:
            yaw_error (float): Error in yaw angle (desired - current)
            distance (float): Distance to target

        Returns:
            float: Commanded surge velocity, limited to minimum speed
        """
        yaw_factor = np.cos(yaw_error)
        distance_factor = min(1.0, distance / self.h_delta_max)
        desired_speed = self.nominal_speed * yaw_factor * distance_factor
        return max(self.min_speed, desired_speed)

    def apply_reference_filter(self, commands: np.ndarray, dt: float) -> np.ndarray:
        """Apply third-order reference filter to guidance commands.

        Filters raw commands through third-order dynamics to ensure smooth
        transitions in position, velocity, and acceleration.

        Args:
            commands (np.ndarray): Raw commands [surge, pitch, yaw]
            dt (float): Time step for integration

        Returns:
            np.ndarray: Filtered position commands [surge, pitch, yaw]
        """
        # Update filter state
        x_dot = self.Ad @ self.x + self.Bd @ commands
        self.x = self.x + x_dot * dt

        # Return filtered position states
        return self.x[0:3]  # Return position states [surge, pitch, yaw]

    def compute_guidance(
        self, current_pos: np.ndarray, target_pos: np.ndarray, dt: float
    ) -> Tuple[np.ndarray, float, float]:
        """Compute filtered guidance commands.

        Main interface method that computes raw LOS guidance commands and
        applies reference filtering for smooth motion.

        Args:
            current_pos (np.ndarray): Current position and orientation [x, y, z, yaw, pitch]
            target_pos (np.ndarray): Target position [x, y, z]
            dt (float): Time step for filter integration

        Returns:
            Tuple containing:
            - np.ndarray: Filtered commands [surge, pitch, yaw]
            - float: Distance to target
            - float: Depth error
        """
        # Step 1: Compute raw LOS guidance
        raw_commands, distance, depth_error = self.compute_raw_los_guidance(
            current_pos, target_pos
        )

        # Step 2: Apply reference filter
        filtered_commands = self.apply_reference_filter(raw_commands, dt)

        # Normalize yaw angle in filtered commands
        filtered_commands[2] = self.ssa(filtered_commands[2])

        return filtered_commands, distance, depth_error

    def reset_filter_state(self, current_commands: np.ndarray) -> None:
        """Reset filter state to initial conditions.

        Resets all filter states while maintaining current position commands
        and zeroing velocities and accelerations.

        Args:
            current_commands (np.ndarray): Current position commands [surge, pitch, yaw]
        """
        self.x = np.zeros(9)
        self.x[0:3] = current_commands  # Initialize positions with current commands
        # Initialize velocities and accelerations to zero
        self.x[3:6] = np.zeros(3)  # velocities
        self.x[6:9] = np.zeros(3)  # accelerations
