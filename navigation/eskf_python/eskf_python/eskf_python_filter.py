# from dataclasses import dataclass
from typing import Tuple

import numpy as np
from eskf_python_class import Measurement, StateEuler, StateQuat
from eskf_python_utils import (
    angle_axis_to_quaternion,
    euler_to_quat,
    quaternion_product,
    skew_matrix,
)
from scipy.linalg import block_diag, expm


class ESKF:
    def __init__(
        self, Q: np.ndarray, P0, nom_state: StateQuat, p_accBias, p_gyroBias, dt
    ):
        self.Q = Q
        self.dt = dt
        self.nom_state = nom_state
        self.error_state = StateEuler()
        self.error_state.covariance = P0
        self.p_accBias = p_accBias
        self.p_gyroBias = p_gyroBias

    def Q_delta_theta(self) -> np.ndarray:
        """Calculates the Q_delta_theta matrix.
        See Joan Solà. Quaternion kinematics for the error-state Kalman filter.
        chapter: 6.1.1 eq. 281
        """
        qw, qx, qy, qz = self.nom_state.orientation

        Q_delta_theta = 0.5 * np.array(
            [
                [-qx, -qy, -qz],
                [qw, -qz, qy],
                [qz, qw, -qx],
                [-qy, qx, qw],
            ]
        )

        return Q_delta_theta

    def Hx(self) -> np.ndarray:
        """Computes the true-state measurement Jacobian for the measurement
            h(x) = R(q) * velocity,
        where:
            - R(q) is the rotation matrix from the quaternion (q, q, q, q) with q as the scalar part,
            - velocity is a 3-vector.

        The state is assumed to be ordered as:
            [position (3), velocity (3), quaternion (4), ...]  (total length 19).

        The Jacobian Hx is a 3x19 matrix with nonzero blocks:
            - Columns 3:6 (velocity): R(q)
            - Columns 6:10 (quaternion):
        """
        q = self.nom_state.orientation  # shape (4,)
        v = self.nom_state.velocity  # shape (3,)
        q0, q1, q2, q3 = q
        v1, v2, v3 = v

        R = self.nom_state.R_q().transpose()  # shape (3, 3)

        dhdq0 = 2 * np.array(
            [
                q0 * v1 - q3 * v2 + q2 * v3,
                q3 * v1 + q0 * v2 - q1 * v3,
                -q2 * v1 + q1 * v2 + q0 * v3,
            ]
        )

        dhdq1 = 2 * np.array(
            [
                q1 * v1 + q2 * v2 + q3 * v3,
                q2 * v1 - q1 * v2 - q0 * v3,
                q3 * v1 + q0 * v2 - q1 * v3,
            ]
        )

        dhdq2 = 2 * np.array(
            [
                -q2 * v1 + q1 * v2 + q0 * v3,
                q1 * v1 + q2 * v2 + q3 * v3,
                -q0 * v1 + q3 * v2 - q2 * v3,
            ]
        )

        dhdq3 = 2 * np.array(
            [
                -q3 * v1 - q0 * v2 + q1 * v3,
                q0 * v1 - q3 * v2 + q2 * v3,
                q1 * v1 + q2 * v2 + q3 * v3,
            ]
        )

        dHdq = np.column_stack((dhdq0, dhdq1, dhdq2, dhdq3))  # shape (3, 4)

        Hx = np.zeros((3, 19))
        Hx[:, 3:6] = R
        Hx[:, 6:10] = dHdq

        return Hx

    def H(self) -> np.ndarray:
        """Calculates the measurement matrix.

        Returns:
            np.ndarray: The measurement matrix.
        """
        X_deltax = block_diag(np.eye(6), self.Q_delta_theta(), np.eye(9))

        H = self.Hx() @ X_deltax

        return H

    def h(self) -> np.ndarray:
        """Calculates the measurement model.

        Returns:
            np.ndarray: The measurement model.
        """
        return self.nom_state.R_q() @ self.nom_state.velocity

    def nominal_state_discrete(self, imu_data: Measurement) -> None:
        """Calculates the next nominal state using the discrete-time process model defined in:
        Joan Solà. Quaternion kinematics for the error-state Kalman filter.
        Chapter: 5.4.1 The nominal state kinematics

        Args:
            imu_data (np.ndarray): The IMU data.
        """
        # Rectify measurements.
        acc_rect = imu_data.acceleration - self.nom_state.acceleration_bias
        gyro_rect = imu_data.angular_velocity - self.nom_state.gyro_bias

        R = self.nom_state.R_q()

        self.nom_state.position = (
            self.nom_state.position
            + self.nom_state.velocity * self.dt
            + 0.5 * (R @ acc_rect + self.nom_state.g) * self.dt**2
        )
        self.nom_state.velocity = (
            self.nom_state.velocity + (R @ acc_rect + self.nom_state.g) * self.dt
        )
        self.nom_state.orientation = quaternion_product(
            self.nom_state.orientation, angle_axis_to_quaternion(gyro_rect * self.dt)
        )
        self.nom_state.acceleration_bias = self.nom_state.acceleration_bias
        self.nom_state.gyro_bias = self.nom_state.gyro_bias
        self.nom_state.g = self.nom_state.g

    def van_loan_discretization(self, A_c, G_c) -> Tuple[np.ndarray, np.ndarray]:
        """Calculates the Van Loan discretization of a continuous-time system.

        Args:
            A_c (np.ndarray): The A matrix.
            G_c (np.ndarray): The G matrix.

        Returns:
            Tuple: The A_d and GQG_d matrices.
        """
        GQG_T = np.dot(np.dot(G_c, self.Q), G_c.T)

        matrix_exp = (
            np.block(
                [
                    [-A_c, GQG_T],
                    [np.zeros((A_c.shape[0], A_c.shape[0])), np.transpose(A_c)],
                ]
            )
            * self.dt
        )

        van_loan_matrix = expm(matrix_exp)

        V1 = van_loan_matrix[A_c.shape[0] :, A_c.shape[0] :]
        V2 = van_loan_matrix[: A_c.shape[0], A_c.shape[0] :]

        A_d = V1.T
        GQG_d = A_d @ V2

        return A_d, GQG_d

    def error_state_prediction(self, imu_data: Measurement) -> None:
        # Rectify measurements.
        acc_rect = imu_data.acceleration - self.nom_state.acceleration_bias
        gyro_rect = imu_data.angular_velocity - self.nom_state.gyro_bias

        R = self.nom_state.R_q()

        A_c = np.zeros((18, 18))

        A_c[0:3, 3:6] = np.eye(3)
        A_c[3:6, 6:9] = -R @ skew_matrix(acc_rect)
        A_c[6:9, 6:9] = -skew_matrix(gyro_rect)
        A_c[3:6, 9:12] = -R
        A_c[9:12, 9:12] = -self.p_accBias * np.eye(3)
        A_c[12:15, 12:15] = -self.p_gyroBias * np.eye(3)
        A_c[6:9, 12:15] = -np.eye(3)
        A_c[3:6, 15:18] = np.eye(3)

        G_c = np.zeros((18, 12))

        G_c[3:6, 0:3] = -R
        G_c[6:9, 3:6] = -np.eye(3)
        G_c[9:12, 6:9] = np.eye(3)
        G_c[12:15, 9:12] = np.eye(3)

        A_d, GQG_d = self.van_loan_discretization(A_c, G_c)

        self.error_state.covariance = A_d @ self.error_state.covariance @ A_d.T + GQG_d

    def measurement_update(self, dvl_measurement: Measurement) -> float:
        """Updates the error state using the DVL measurement.
        Joan Solà. Quaternion kinematics for the error-state Kalman filter.
        Chapter: 6.1 eq. 274-276

        Args:
            dvl_measurement (np.ndarray): The DVL measurement.
        """
        H = self.H()
        P = self.error_state.covariance
        R = dvl_measurement.aiding_covariance

        S = H @ P @ H.T + R
        K = P @ H.T @ np.linalg.inv(S)
        innovation = dvl_measurement.aiding - self.h()

        NIS_value = self.NIS(S, innovation)

        self.error_state.fill_states(K @ innovation)

        I_KH = np.eye(18) - K @ H
        self.error_state.covariance = (
            I_KH @ P @ I_KH.T + K @ R @ K.T
        )  # Joseph form for more stability
        return NIS_value

    def injection(self) -> None:
        """Injects the error state into the nominal state to produce the estimated state.
        Joan Solà. Quaternion kinematics for the error-state Kalman filter.
        Chapter 6.2 eq. 282-283

        """
        self.nom_state.position = self.nom_state.position + self.error_state.position
        self.nom_state.velocity = self.nom_state.velocity + self.error_state.velocity
        self.nom_state.orientation = quaternion_product(
            self.nom_state.orientation, euler_to_quat(self.error_state.orientation)
        )
        self.nom_state.acceleration_bias = (
            self.nom_state.acceleration_bias + self.error_state.acceleration_bias
        )
        self.nom_state.gyro_bias = self.nom_state.gyro_bias + self.error_state.gyro_bias
        self.nom_state.g = self.nom_state.g + self.error_state.g

    def reset_error_state(self) -> None:
        """Resets the error state after injection.
        Joan Solà. Quaternion kinematics for the error-state Kalman filter.
        Chapter 6.3 eq. 284-286
        """
        G = np.eye(18)  # Neglecting the delta_theta as this is most common in practice

        self.error_state.covariance = G @ self.error_state.covariance @ G.T
        self.error_state.fill_states(np.zeros(18))

    def imu_update(self, imu_data: Measurement) -> None:
        """Updates the state using the IMU data."""
        self.nominal_state_discrete(imu_data)
        self.error_state_prediction(imu_data)

    def dvl_update(self, dvl_measurement: Measurement) -> float:
        """Updates the state using the DVL measurement."""
        NIS = self.measurement_update(dvl_measurement)
        self.injection()
        self.reset_error_state()

        return NIS

    # functions for tuning the filter
    def NIS(self, S: np.ndarray, innovation: np.ndarray) -> float:
        """Calculates the Normalized Innovation Squared (NIS) value."""
        return innovation.T @ np.linalg.inv(S) @ innovation

    def NEEDS(
        self, P: np.ndarray, true_state: StateQuat, estimate_state: StateQuat
    ) -> float:
        """Calculates the Normalized Estimation Error Squared (NEEDS) value."""
        return (
            (true_state - estimate_state).as_vector().T
            @ np.linalg.inv(P)
            @ (true_state - estimate_state).as_vector()
        )
