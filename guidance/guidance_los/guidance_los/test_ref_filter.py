#test_ref_filter.py

#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt

class FilteredGuidanceCalculator:
    def __init__(self):
        # State vector dimensions: [pos, vel, acc] for [surge, pitch, yaw]
        self.x = np.zeros(9)  # State vector for reference filter
        
        # Filter parameters
        self.omega = np.diag([1.8, 1.8, 1.8])  # Natural frequency
        self.zeta = np.diag([0.8, 0.8, 0.8])   # Damping ratio
        
        # Initialize matrices
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

    def compute_filtered_guidance(self, target_pos: np.ndarray, dt: float) -> np.ndarray:
        """Compute filtered guidance using only position states."""
        # Update filter state
        x_dot = self.Ad @ self.x + self.Bd @ target_pos
        self.x = self.x + x_dot * dt
        
        # Return only position states
        return self.x[0:3] # what cyprian gets

def test_filtered_guidance_response():
    calculator = FilteredGuidanceCalculator()
    
    # Define initial and target positions as numpy arrays
    target_pos = np.array([10.0, 3.14, 3.14])  # Example target position
    
    # Simulation parameters
    dt = 0.01
    total_time = 5.0
    num_steps = int(total_time / dt)
    
    # Data storage
    position_data = np.zeros((num_steps, 3))
    time_data = np.arange(0, total_time, dt)

    # Simulation loop
    for step in range(num_steps):
        # Get filtered position response
        position_data[step] = calculator.compute_filtered_guidance(target_pos, dt)
        current_pos = position_data[step]

    # Plot results
    plt.figure(figsize=(12, 8))
    labels = ['Surge', 'Pitch', 'Yaw']
    
    for i in range(3):
        plt.subplot(3, 1, i+1)
        plt.plot(time_data, position_data[:, i], label=f'{labels[i]} Response')
        plt.xlabel('Time (s)')
        plt.ylabel(f'{labels[i]} Position')
        plt.legend()
        plt.grid()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    test_filtered_guidance_response()