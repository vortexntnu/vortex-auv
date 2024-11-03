#filtered_guidance_calculator.py

#!/usr/bin/env python3

import numpy as np
from typing import Dict, Any

class FilteredGuidanceCalculator:
    def __init__(self):
        # Initialize LOS parameters
        self.h_delta_min = 1.0
        self.h_delta_max = 5.0
        self.h_delta_factor = 3.0
        self.nominal_speed = 0.35
        self.min_speed = 0.1
        self.max_pitch_angle = np.pi/6
        self.depth_gain = 0.5

        # State vector dimensions: [pos, vel, acc] for [surge, pitch, yaw]
        self.x = np.zeros(9)  # State vector for reference filter
        
        # Filter parameters (can be tuned)
        self.omega = np.diag([0.1, 0.5, 0.1])  # Natural frequency
        self.zeta = np.diag([1, 1, 1])   # Damping ratio
        
        # Initialize matrices
        self.setup_filter_matrices()

    def setup_filter_matrices(self):
        """Setup state-space matrices for the reference filter."""
        # System matrices for third-order reference filter (9 states, 3 inputs)
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
        
        # Fill Bd matrix
        self.Bd[6:9, :] = omega_cubed

    def normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def compute_los_guidance(self, current_pos: Dict[str, float], target_pos: Dict[str, float]) -> Dict[str, float]:
        """Compute raw LOS guidance commands."""
        dx = target_pos['x'] - current_pos['x']
        dy = target_pos['y'] - current_pos['y']
        
        horizontal_distance = np.sqrt(dx**2 + dy**2)
        desired_yaw = np.arctan2(dy, dx)
        yaw_error = self.normalize_angle(desired_yaw - current_pos['yaw'])
        
        depth_error = target_pos['z'] - current_pos['z']
        desired_pitch = self.compute_pitch_command(depth_error, horizontal_distance)
        desired_surge = self.compute_desired_speed(yaw_error, horizontal_distance)

        return {
            'surge': desired_surge,
            'pitch': desired_pitch,
            'yaw': desired_yaw,
            'distance': horizontal_distance,
            'depth_error': depth_error
        }

    def compute_filtered_guidance(self, current_pos: Dict[str, float], target_pos: Dict[str, float], dt: float) -> Dict[str, float]:
        """Compute filtered guidance commands."""
        # Get raw LOS guidance
        los_commands = self.compute_los_guidance(current_pos, target_pos)
        
        # Create reference vector (only 3 inputs now: surge, pitch, yaw)
        r = np.array([
            los_commands['surge'],
            los_commands['pitch'],
            los_commands['yaw']
        ])
        
        # Update filter state
        x_dot = self.Ad @ self.x + self.Bd @ r
        self.x = self.x + x_dot * dt
        
        # Return filtered commands
        return {
            'surge': self.x[3],  # Use velocity states
            'pitch': self.x[4],
            'yaw': self.x[5],
            'distance': los_commands['distance'],
            'depth_error': los_commands['depth_error']
        }
    
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

    def reset_filter_state(self, current_pos: Dict[str, float]) -> None:
        """Reset filter state to current position."""
        self.x = np.zeros(9)
        # Initialize positions (only 3 DOF)
        self.x[0:3] = np.array([
            current_pos.get('surge', 0.0),
            current_pos.get('pitch', 0.0),
            current_pos.get('yaw', 0.0)
        ])