import numpy as np
from ukf_okid_class import StateQuat


def print_StateQuat_list(
    state_list: list[StateQuat], name="StateQuat List", print_covariance=True
):
    """Custom print function to print a list of StateQuat objects in a formatted form."""
    print(f"{name}:")
    for i, state in enumerate(state_list):
        print(f"Index {i}:")
        print_StateQuat(state, f"StateQuat {i}", print_covariance)


def print_StateQuat(state: StateQuat, name="StateQuat", print_covariance=True):
    """Custom print function to print StateQuat objects in a formatted form."""
    print(f"{name}:")
    print(f"  Position: {state.position}")
    print(f"  Orientation: {state.orientation}")
    print(f"  Velocity: {state.velocity}")
    print(f"  Angular Velocity: {state.angular_velocity}")
    # print(f"  okid_params: {state.okid_params}")
    if print_covariance:
        print_matrix(state.covariance, "Covariance")


def print_matrix(matrix, name="Matrix"):
    """Custom print function to print matrices in a formatted form."""
    print(f"{name}: {matrix.shape}")
    if isinstance(matrix, np.ndarray):
        for row in matrix:
            print(" ".join(f"{val:.2f}" for val in row))
    else:
        print(matrix)
