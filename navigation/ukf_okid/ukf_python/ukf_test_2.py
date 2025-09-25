import numpy as np

# Define process noise covariance Q (33x33)
Q = np.zeros((12, 12))
Q[0:3, 0:3] = np.eye(3) * 0.003  # for position
Q[3:6, 3:6] = np.eye(3) * 0.003  # for orientation error (represented with Euler angles)
Q[6:9, 6:9] = np.eye(3) * 0.002  # for velocity
Q[9:12, 9:12] = np.eye(3) * 0.003  # for angular velocity

G = np.zeros((33, 12))
G[0:3, 0:3] = np.eye(3)
G[3:6, 3:6] = np.eye(3)
G[6:9, 6:9] = np.eye(3)
G[9:12, 9:12] = np.eye(3)

GG = G @ Q @ G.T


def fancy_print_matrix(matrix, name="Matrix", precision=4):
    """Print a matrix with fancy formatting.

    Args:
        matrix: numpy array to print
        name: name of the matrix to display
        precision: number of decimal places to show
    """
    print(f"\n{'=' * 50}")
    print(f" {name} [{matrix.shape[0]}x{matrix.shape[1]}]")
    print(f"{'=' * 50}")

    # Set numpy print options
    with np.printoptions(precision=precision, suppress=True, linewidth=100):
        # Print each row with custom formatting
        for i in range(matrix.shape[0]):
            row = ' '.join([f"{x:8.{precision}f}" for x in matrix[i]])
            print(f" {i:2d} | {row}")

    print(f"{'=' * 50}\n")


# Example usage:
fancy_print_matrix(GG, name="Process Noise Covariance (GQG')", precision=3)
