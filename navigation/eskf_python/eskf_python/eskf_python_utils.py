import numpy as np

def skew_matrix(vector: np.ndarray) -> np.ndarray:
    """
    Returns the skew symmetric matrix of a 3x1 vector.
    """
    return np.array(
        [
            [0, -vector[2], vector[1]],
            [vector[2], 0, -vector[0]],
            [-vector[1], vector[0], 0]
        ]
    )

def quaternion_product(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
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

        eta_new = eta_0 * eta_1 - np.dot(e_0, e_1)
        nu_new = e_1 * eta_0 + e_0 * eta_1 + np.cross(e_0, e_1)

        q_new = np.array([eta_new, nu_new[0], nu_new[1], nu_new[2]])
        q_new = q_new / np.linalg.norm(q_new)

        return q_new

def angle_axis_to_quaternion(vector: np.ndarray) -> np.ndarray:
    """Converts an angle-axis representation to a quaternion.

    Args:
        vector (np.ndarray): The angle-axis representation.

    Returns:
        np.ndarray: The quaternion representation.
    """
    angle = np.linalg.norm(vector)
    if angle < 1e-8:
        return np.array([1, 0, 0, 0])
    else:
        axis = vector / angle


        q = np.zeros(4)
        q[0] = np.cos(angle / 2)
        q[1:] = np.sin(angle / 2) * axis

        return q
        

def R_from_angle_axis(vector: np.ndarray) -> np.ndarray:
    """Calculates the rotation matrix from the angle-axis representation.

    Args:
        vector (np.ndarray): The angle-axis representation.

    Returns:
        np.ndarray: The rotation matrix.
    """
    quaternion = angle_axis_to_quaternion(vector)
    q0, q1, q2, q3 = quaternion

    R = np.array(
        [
            [
                1 - 2 * q2**2 - 2 * q3**2,
                2 * (q1 * q2 - q0 * q3),
                2 * (q0 * q2 + q1 * q3),
            ],
            [
                2 * (q1 * q2 + q0 * q3),
                1 - 2 * q1**2 - 2 * q3**2,
                2 * (q2 * q3 - q0 * q1),
            ],
            [
                2 * (q1 * q3 - q0 * q2),
                2 * (q0 * q1 + q2 * q3),
                1 - 2 * q1**2 - 2 * q2**2,
            ],
        ]
    )  

    return R
