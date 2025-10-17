import numpy as np
from velocity_controller_lqr.velocity_controller_lqr_lib import (
    LQRController,
    LQRParameters,
)
from vortex_utils.python_utils import State

lqr_params = LQRParameters(
    q_surge=1.0,
    q_pitch=1.0,
    q_yaw=1.0,
    r_surge=1.0,
    r_pitch=1.0,
    r_yaw=1.0,
    i_surge=1.0,
    i_pitch=1.0,
    i_yaw=1.0,
    i_weight=1.0,
    max_force=50.0,
)
controller = LQRController(lqr_params, np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]))


def calculate_coriolis(state: State) -> np.ndarray:
    return controller.calculate_coriolis_matrix(
        pitch_rate=state.twist.angular_y,
        yaw_rate=state.twist.angular_z,
        sway_vel=state.twist.linear_y,
        heave_vel=state.twist.linear_z,
    )


class TestVelocityController:
    def test_placeholder(self):
        assert controller is not None

    def test_saturate(self):
        print("Commencing saturate test: \n")

        # Test case 1: Saturation occurs
        windup, saturated_value = controller.saturate(10, False, 5)
        assert saturated_value == 5.0
        assert windup == True

        # Test case 2: Saturation occurs with negative limit
        windup, saturated_value = controller.saturate(-10, False, 5)
        assert saturated_value == -5.0
        assert windup == True

        # Test case 3: No saturation
        windup, saturated_value = controller.saturate(3, True, 5)
        assert saturated_value == 3.0
        assert windup == False

        # Test case 4: No saturation with negative value
        windup, saturated_value = controller.saturate(-3, True, 5)
        assert saturated_value == -3.0
        assert windup == False

        print("Saturate test passed")

    def test_anti_windup(self):
        print("Commencing anti windup test: \n")

        windup = True
        assert controller.anti_windup(10, 5, 10, windup) == 10

        windup = False
        assert controller.anti_windup(1, 5, 10, windup) == 15

        print("Anti windup test passed")

    def test_max_force(self):
        assert 0 <= controller.max_force <= 99.9, (
            "Max force must be in the range [0, 99.9]."
        )

    def test_zero_command(self):
        state = State()
        guidance = State()
        u = controller.calculate_lqr_u(state=state, guidance_values=guidance)
        assert np.allclose(u, 0.0, atol=1e-9)

    def test_positive_surge_command(self):
        state = State()
        guidance = State()
        guidance.twist.linear_x = 1.0
        u = controller.calculate_lqr_u(state=state, guidance_values=guidance)
        assert np.greater(u[0], 0.0)
        assert np.allclose(u[1:], 0.0, atol=1e-9)

    def test_negative_surge_command(self):
        state = State()
        guidance = State()
        guidance.twist.linear_x = -1.0
        u = controller.calculate_lqr_u(state=state, guidance_values=guidance)
        assert np.less(u[0], 0.0)
        assert np.allclose(u[1:], 0.0, atol=1e-9)

    def test_positive_pitch_command(self):
        state = State()
        guidance = State()
        guidance.pose.pitch = 1.0
        u = controller.calculate_lqr_u(state=state, guidance_values=guidance)
        assert np.greater(u[1], 0.0)
        assert np.isclose(u[0], 0.0, atol=1e-9)
        assert np.isclose(u[2], 0.0, atol=1e-9)

    def test_negative_pitch_command(self):
        state = State()
        guidance = State()
        guidance.pose.pitch = -1.0
        u = controller.calculate_lqr_u(state=state, guidance_values=guidance)
        assert np.less(u[1], 0.0)
        assert np.isclose(u[0], 0.0, atol=1e-9)
        assert np.isclose(u[2], 0.0, atol=1e-9)

    def test_positive_yaw_command(self):
        state = State()
        guidance = State()
        guidance.pose.yaw = 1.0
        u = controller.calculate_lqr_u(state=state, guidance_values=guidance)
        assert np.greater(u[2], 0.0)
        assert np.isclose(u[0], 0.0, atol=1e-9)
        assert np.isclose(u[1], 0.0, atol=1e-9)

    def test_negative_yaw_command(self):
        state = State()
        guidance = State()
        guidance.pose.yaw = -1.0
        u = controller.calculate_lqr_u(state=state, guidance_values=guidance)
        assert np.less(u[2], 0.0)
        assert np.isclose(u[0], 0.0, atol=1e-9)
        assert np.isclose(u[1], 0.0, atol=1e-9)

    def test_yaw_ssa(self):
        state = State()
        state.pose.yaw = 6.10  # rad
        guidance = State()
        guidance.pose.yaw = 0.1  # rad
        u = controller.calculate_lqr_u(
            state=state, guidance_values=guidance
        )  # command should be slightly positive (to the right)
        assert np.greater(u[2], 0.0)
