import numpy as np
import math

class LOSGuidanceCalculator:
    def __init__(self):
        # Horizontal guidance parameters
        self.h_delta_min = 1.0      # Minimum look-ahead distance
        self.h_delta_max = 5.0      # Maximum look-ahead distance
        self.h_delta_factor = 3.0   # Look-ahead scaling factor
        
        # Speed control parameters
        self.nominal_speed = 0.35   # Nominal surge velocity
        self.min_speed = 0.1        # Minimum surge velocity
        
        # Vertical control parameters
        self.max_pitch_angle = np.pi/6  # 30 degrees max pitch
        self.depth_gain = 0.5           # Gain for depth error

    def normalize_angle(self, angle):
        """Normalize angle to be within [-pi, pi]."""
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def compute_los_guidance(self, current_pos, target_pos):
        """
        Compute LOS guidance commands for surge, pitch, and yaw.
        
        Args:
            current_pos: dict with x, y, z, yaw, pitch
            target_pos: dict with x, y, z
        
        Returns:
            dict with desired surge velocity, yaw angle, and pitch angle
        """
        # Horizontal plane calculations
        dx = target_pos['x'] - current_pos['x']
        dy = target_pos['y'] - current_pos['y']
        
        # Distance to target in horizontal plane
        horizontal_distance = math.sqrt(dx**2 + dy**2)
        
        # Desired yaw (LOS angle)
        desired_yaw = math.atan2(dy, dx)
        yaw_error = self.normalize_angle(desired_yaw - current_pos['yaw'])
        
        # Compute adaptive look-ahead distance for horizontal plane
        h_delta = self.compute_adaptive_lookahead(horizontal_distance)
        
        # Vertical plane calculations
        depth_error = target_pos['z'] - current_pos['z']
        
        # Compute desired pitch based on depth error
        # Using a simplified proportional control with saturation
        desired_pitch = self.compute_pitch_command(depth_error, horizontal_distance)
        
        # Compute desired surge velocity
        desired_surge = self.compute_desired_speed(yaw_error, horizontal_distance)
        
        return {
            'surge': desired_surge,
            'yaw': desired_yaw,
            'pitch': desired_pitch,
            'distance': horizontal_distance,
            'depth_error': depth_error
        }

    def compute_adaptive_lookahead(self, distance):
        """Compute adaptive look-ahead distance based on distance to target."""
        delta = self.h_delta_factor * distance
        return np.clip(delta, self.h_delta_min, self.h_delta_max)

    def compute_desired_speed(self, yaw_error, distance):
        """
        Compute desired surge velocity based on yaw error and distance.
        Reduces speed for large yaw errors and when close to target.
        """
        # Reduce speed based on yaw error
        yaw_factor = math.cos(yaw_error)  # cos gives smooth reduction for large errors
        
        # Reduce speed when close to target
        distance_factor = min(1.0, distance / self.h_delta_max)
        
        # Compute final speed
        desired_speed = self.nominal_speed * yaw_factor * distance_factor
        return max(self.min_speed, desired_speed)

    def compute_pitch_command(self, depth_error, horizontal_distance):
        """
        Compute desired pitch angle based on depth error and horizontal distance.
        Adjusts pitch commands based on how far we are from target.
        """
        # Basic proportional control on depth error
        raw_pitch = -self.depth_gain * depth_error  # Negative because positive pitch decreases depth
        
        # Modify pitch based on horizontal distance
        # Reduce pitch angles when close to target
        distance_factor = min(1.0, horizontal_distance / self.h_delta_max)
        modified_pitch = raw_pitch * distance_factor
        
        # Saturate pitch command
        return np.clip(modified_pitch, -self.max_pitch_angle, self.max_pitch_angle)

    def get_debug_info(self, guidance_commands):
        """Return formatted debug information."""
        return {
            'Surge velocity': f"{guidance_commands['surge']:.2f} m/s",
            'Yaw angle': f"{math.degrees(guidance_commands['yaw']):.1f} deg",
            'Pitch angle': f"{math.degrees(guidance_commands['pitch']):.1f} deg",
            'Distance to target': f"{guidance_commands['distance']:.2f} m",
            'Depth error': f"{guidance_commands['depth_error']:.2f} m"
        }