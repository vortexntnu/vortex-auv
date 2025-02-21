import numpy as np

def quaternion_super_product(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
        """Calculates the quaternion super product of two quaternions.

        Args:
            q1 (np.ndarray): The first quaternion.
            q2 (np.ndarray): The second quaternion.

        Returns:
            np.ndarray: The quaternion super product.
        """
        eta_0, e_0_x, e_0_y, e_0_z = q1
        eta_1, e_1_x, e_1_y, e_1_z = q2

        e_0 = np.array([e_0_x, e_0_y, e_0_z])
        e_1 = np.array([e_1_x, e_1_y, e_1_z])

        eta_new = eta_0 * eta_1 - (e_0_x * e_1_x + e_0_y * e_1_y + e_0_z * e_1_z)
        nu_new = e_1 * eta_0 + e_0 * eta_1 + np.dot(skew_symmetric(e_0), e_1)

        q_new = np.array([eta_new, nu_new[0], nu_new[1], nu_new[2]])
        q_new /= np.linalg.norm(q_new)

        return q_new
    
def quaternion_error(quat_1: np.ndarray, quat_2: np.ndarray) -> np.ndarray:
    """
    Calculates the error between two quaternions
    """

    quat_2_inv = np.array([quat_2[0], -quat_2[1], -quat_2[2], -quat_2[3]])

    error_quat = quaternion_super_product(quat_1, quat_2_inv)

    return error_quat

def euler_rotation_quaternion(self, euler_angles: np.ndarray) -> np.ndarray:
        """
        Converts An vector assumed to be rotation vector to quaternion
        
        Args:
            euler_angles (np.ndarray): Rotation vector
        
        Returns:
            np.ndarray: Quaternion representation of the rotation vector
        """

        angle = np.linalg.norm(euler_angles)

        if angle == 0:
            axis = np.array([0, 0, 0])
        else:
            axis = euler_angles / angle

        quaternion = np.zeros(4)
        quaternion[0] = np.cos(angle / 2)
        quaternion[1:] = np.sin(angle / 2) * axis

        return quaternion

def quaternion_rotation_euler(self, quaternion: np.ndarray) -> np.ndarray:
    """
    Converts a quaternion to an euler rotation vector
    Used to generate the covarince matrix

    Args:
        quaternion (np.ndarray): The quaternion to convert
    
    Returns:
        np.ndarray: The euler rotation vector
    """
    nu, eta_x, eta_y, eta_z = quaternion
    
    phi = np.arctan2(2 * (nu * eta_x + eta_y * eta_z), 1 - 2 * (eta_x ** 2 + eta_y ** 2))
    theta = -np.arcsin(2 * (eta_z * eta_x - nu * eta_y))
    psi = np.arctan2(2 * (nu * eta_z + eta_x * eta_y), 1 - 2 * (eta_y ** 2 + eta_z ** 2))

    return np.array([phi, theta, psi])

def skew_symmetric(vector: np.ndarray) -> np.ndarray:
        """Calculates the skew symmetric matrix of a vector.

        Args:
            vector (np.ndarray): The vector.

        Returns:
            np.ndarray: The skew symmetric matrix.
        """
        return np.array(
            [
                [0, -vector[2], vector[1]],
                [vector[2], 0, -vector[0]],
                [-vector[1], vector[0], 0],
            ]
        )

def ssa(angle: np.ndarray) -> np.ndarray:
    """
    smallest signed angle between two angles
    """
    ssa_vector = np.zeros(len(angle))

    for i in range(len(angle)):
        ssa_vector[i] = (angle[i] + np.pi) % (2 * np.pi) - np.pi

    return ssa_vector