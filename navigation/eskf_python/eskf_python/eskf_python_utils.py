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

def quat_norm(quat: np.ndarray) -> np.ndarray:
    """
    Function that normalizes a quaternion
    """
    quat = quat / np.linalg.norm(quat)

    return quat

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

def quaternion_error(quat_1: np.ndarray, quat_2: np.ndarray) -> np.ndarray:
    """
    Calculates the error between two quaternions
    """
    
    quat_2_inv = np.array([quat_2[0], -quat_2[1], -quat_2[2], -quat_2[3]])

    error_quat = quaternion_product(quat_1, quat_2_inv)

    return error_quat

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

def euler_to_quat(euler_angles: np.ndarray) -> np.ndarray:
    """
    Converts Euler angles to a quaternion
    """
    psi, theta, phi = euler_angles
    c_psi = np.cos(psi / 2)
    s_psi = np.sin(psi / 2) 
    c_theta = np.cos(theta / 2)
    s_theta = np.sin(theta / 2)
    c_phi = np.cos(phi / 2)
    s_phi = np.sin(phi / 2)

    quat = np.array([
        c_psi * c_theta * c_phi + s_psi * s_theta * s_phi,
        c_psi * c_theta * s_phi - s_psi * s_theta * c_phi,
        s_psi * c_theta * s_phi + c_psi * s_theta * c_phi,
        s_psi * c_theta * c_phi - c_psi * s_theta * s_phi
    ])

    return quat

def quat_to_euler(quat: np.ndarray) -> np.ndarray:
    """
    Converts a quaternion to Euler angles
    """
    nu, eta_1, eta_2, eta_3 = quat

    phi = np.arctan2(2*(eta_2 * eta_3 + nu * eta_1), 1 - 2 * (eta_1 ** 2 + eta_2 ** 2))
    theta = -np.arcsin(2 * (eta_1 * eta_3 - nu * eta_2))
    psi = np.arctan2(2 * (nu * eta_3 + eta_1 * eta_2), 1 - 2 * (eta_2 ** 2 + eta_3 ** 2))

    return np.array([phi, theta, psi])

