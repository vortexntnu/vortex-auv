# integrated_los_guidance.py

#!/usr/bin/env python3

import numpy as np
from typing import Tuple

class IntegratedLOSGuidance:
    def __init__(self):
        # LOS parameters
        self.h_delta_min = 1.0      # Minimum look-ahead distance
        self.h_delta_max = 5.0      # Maximum look-ahead distance
        self.h_delta_factor = 3.0   # Look-ahead scaling factor
        self.nominal_speed = 0.35   # Nominal surge velocity
        self.min_speed = 0.1        # Minimum surge velocity
        self.max_pitch_angle = np.pi/6  # 30 degrees max pitch
        self.depth_gain = 0.5       # Gain for depth error

        # Filter parameters
        self.x = np.zeros(9)  # State vector for reference filter [pos, vel, acc] for [surge, pitch, yaw]
        self.omega = np.diag([1.8, 1.8, 1.8])  # Natural frequency
        self.zeta = np.diag([0.8, 0.8, 0.8])   # Damping ratio
        
        # Initialize filter matrices
        self.setup_filter_matrices()

    def setup_filter_matrices(self):
        """Setup state-space matrices for the reference filter."""
        self.Ad = np.zeros((9, 9))
        self.Bd = np.zeros((9, 3))
        
        # Fill Ad matrix blocks
        self.Ad[0:3, 3:6] = np.eye(3)  # Position to velocity
        self.Ad[3:6, 6:9] = np.eye(3)  # Velocity to acceleration
        
        omega_cubed = self.omega @ self.omega @ self.omega
        omega_squared = self.omega @ self.omega
        
        self.Ad[6:9, 0:3] = -omega_cubed
        self.Ad[6:9, 3:6] = -(2 * self.zeta + np.eye(3)) @ omega_squared
        self.Ad[6:9, 6:9] = -(2 * self.zeta + np.eye(3)) @ self.omega
        
        self.Bd[6:9, :] = omega_cubed

    def normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def compute_raw_los_guidance(self, current_pos: np.ndarray, 
                               target_pos: np.ndarray) -> Tuple[np.ndarray, float, float]:
        """
        Compute raw LOS guidance commands.
        
        Args:
            current_pos: np.ndarray [x, y, z, yaw, pitch]
            target_pos: np.ndarray [x, y, z]
            
        Returns:
            Tuple containing:
            - np.ndarray [surge, pitch, yaw]: commanded values
            - float: distance to target
            - float: depth error
        """
        # Extract positions
        dx = target_pos[0] - current_pos[0]  # x
        dy = target_pos[1] - current_pos[1]  # y
        
        # Horizontal plane calculations
        horizontal_distance = np.sqrt(dx**2 + dy**2)
        desired_yaw = np.arctan2(dy, dx)
        yaw_error = self.normalize_angle(desired_yaw - current_pos[3])  # current_pos[3] is yaw
        
        # Vertical plane calculations
        depth_error = target_pos[2] - current_pos[2]  # z
        desired_pitch = self.compute_pitch_command(depth_error, horizontal_distance)
        
        # Compute desired surge velocity
        desired_surge = self.compute_desired_speed(yaw_error, horizontal_distance)

        # Return commands as numpy array
        commands = np.array([desired_surge, desired_pitch, desired_yaw])
        return commands, horizontal_distance, depth_error

    def compute_pitch_command(self, depth_error: float, horizontal_distance: float) -> float:
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
        """
        Apply reference filter to guidance commands.
        
        Args:
            commands: np.ndarray [surge, pitch, yaw]
            dt: float time step
            
        Returns:
            np.ndarray: filtered commands [surge, pitch, yaw]
        """
        # Update filter state
        x_dot = self.Ad @ self.x + self.Bd @ commands
        self.x = self.x + x_dot * dt
        
        # Return filtered velocity states
        return self.x[3:6]  # Return velocity states [surge, pitch, yaw]

    def compute_guidance(self, current_pos: np.ndarray, 
                        target_pos: np.ndarray, 
                        dt: float) -> Tuple[np.ndarray, float, float]:
        """
        Compute filtered guidance commands.
        
        Args:
            current_pos: np.ndarray [x, y, z, yaw, pitch]
            target_pos: np.ndarray [x, y, z]
            dt: float time step
            
        Returns:
            Tuple containing:
            - np.ndarray [surge, pitch, yaw]: filtered commands
            - float: distance to target
            - float: depth error
        """
        # Step 1: Compute raw LOS guidance
        raw_commands, distance, depth_error = self.compute_raw_los_guidance(current_pos, target_pos)
        
        # Step 2: Apply reference filter
        filtered_commands = self.apply_reference_filter(raw_commands, dt)
        
        # Normalize yaw angle in filtered commands
        filtered_commands[2] = self.normalize_angle(filtered_commands[2])
        
        return filtered_commands, distance, depth_error

    def reset_filter_state(self, current_commands: np.ndarray) -> None:
        """
        Reset filter state.
        
        Args:
            current_commands: np.ndarray [surge, pitch, yaw]
        """
        self.x = np.zeros(9)
        self.x[0:3] = current_commands  # Initialize positions with current commands