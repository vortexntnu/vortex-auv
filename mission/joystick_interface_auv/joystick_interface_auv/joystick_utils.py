from dataclasses import dataclass
import numpy as np

class JoyStates:
    XBOX_MODE = "XBOX_MODE"
    AUTONOMOUS_MODE = "AUTONOMOUS_MODE"
    KILLSWITCH = "KILLSWITCH"
    REFERENCE_MODE = "REFERENCE_MODE"

@dataclass(slots=True)
class State:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

class Wired:
    joystick_buttons_map_ = [
        "A",
        "B",
        "X",
        "Y",
        "LB",
        "RB",
        "back",
        "start",
        "power",
        "stick_button_left",
        "stick_button_right",
        "share_button",
]

    joystick_axes_map_ = [
        "horizontal_axis_left_stick", #Sway
        "vertical_axis_left_stick", #Surge
        "LT", #Negative thrust/torque multiplier
        "horizontal_axis_right_stick", #Yaw
        "vertical_axis_right_stick",
        "RT", #Positive thrust/torque multiplier
        "dpad_horizontal",
        "dpad_vertical",
]

class WirelessXboxSeriesX:
    joystick_buttons_map_ = [
        "A",
        "B",
        "0",
        "X",
        "Y",
        "0",
        "LB",
        "RB",
        "0",
        "0",
        "back",
        "start",
        "power",
        "stick_button_left",
        "stick_button_right",
        "share_button",
]

    joystick_axes_map_ = [
        "horizontal_axis_left_stick", #Sway
        "vertical_axis_left_stick", #Surge
        "horizontal_axis_right_stick", #Yaw
        "vertical_axis_right_stick",
        "RT", #Positive thrust/torque multiplier
        "LT", #Negative thrust/torque multiplier
        "dpad_horizontal",
        "dpad_vertical",
]
    
def euler_to_quat(roll: float, pitch: float, yaw: float) -> np.ndarray:
        """
        Converts Euler angles (roll, pitch, yaw) to a quaternion.

        This function converts the provided Euler angles (in radians) to a quaternion
        representation. The quaternion is returned as a NumPy array with four elements:
        [w, x, y, z].

        Args:
            roll (float): The roll angle in radians.
            pitch (float): The pitch angle in radians.
            yaw (float): The yaw angle in radians.

        Returns:
            np.ndarray: A NumPy array representing the quaternion [w, x, y, z].
        """
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        w = cy * cp * cr + sy * sp * sr
        x = cy * cp * sr - sy * sp * cr
        y = sy * cp * sr + cy * sp * cr
        z = sy * cp * cr - cy * sp * sr

        return np.array([w, x, y, z])

def skew_symmetric(vector: np.ndarray) -> np.ndarray:
        """
        Computes the skew-symmetric matrix of a 3D vector.

        This function computes the skew-symmetric matrix of the provided 3D vector.
        The skew-symmetric matrix is returned as a NumPy array with shape (3, 3).

        Args:
            vector (np.ndarray): A NumPy array representing a 3D vector.

        Returns:
            np.ndarray: A NumPy array representing the skew-symmetric matrix.
        """
        return np.array([[0.0, -vector[2], vector[1]],
                         [vector[2], 0.0, -vector[0]],
                         [-vector[1], vector[0], 0.0]])

def rotation_matrix_from_quat(quat: np.ndarray) -> np.ndarray:
        """
        Computes the rotation matrix from a quaternion.

        This function computes the rotation matrix from the provided quaternion.
        The quaternion is expected to be a NumPy array with four elements: [w, x, y, z].
        The rotation matrix is returned as a NumPy array with shape (3, 3).

        Args:
            quat (np.ndarray): A NumPy array representing the quaternion [w, x, y, z].

        Returns:
            np.ndarray: A NumPy array representing the rotation matrix.
        """
        tol = 1e-6
        if np.abs(np.linalg.norm(quat) - 1.0) > tol:
            raise ValueError("Quaternion must be normalized.")
        eta = quat[0]
        eps = quat[1:]

        S = skew_symmetric(eps)
        R = np.eye(3) + 2.0 * eta * S + 2.0 * np.dot(S, S)

        return R

def quat_to_euler(quat: np.ndarray) -> np.ndarray:
        """
        Converts a quaternion to Euler angles (roll, pitch, yaw).

        This function converts the provided quaternion to Euler angles (in radians).
        The quaternion is expected to be a NumPy array with four elements: [w, x, y, z].

        Args:
            quat (np.ndarray): A NumPy array representing the quaternion [w, x, y, z].

        Returns:
            np.ndarray: A NumPy array representing the Euler angles [roll, pitch, yaw].
        """
        quat = quat / np.linalg.norm(quat)
        R = rotation_matrix_from_quat(quat)

        phi = np.arctan2(R[2, 1], R[2, 2])
        theta = -np.arcsin(R[2, 0])
        psi = np.arctan2(R[1, 0], R[0, 0])

        return np.array([phi, theta, psi])