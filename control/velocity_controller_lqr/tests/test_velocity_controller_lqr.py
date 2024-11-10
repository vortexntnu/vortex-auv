import numpy as np
from velocity_controller_lqr.velocity_controller_lqr_lib import LQRController

controller = LQRController(0, 0, 0, 0, 0, 0, 0, 0, 0, 0)


class TestVelocityController:
    def test_placeholder(self):
        assert (
            controller is not None
        )  # Simple test to ensure the controller initializes

    def test_ssa(self):
        print("Commencing ssa test: \n")

        assert LQRController.ssa(np.pi + 1) == -np.pi + 1

        assert LQRController.ssa(-np.pi - 1) == (np.pi - 1)

        print("SSA test passed")

    def test_quaternion_to_euler_angle(self):
        print("Commencing quaternion to euler angle test: \n")

        roll, pitch, yaw = LQRController.quaternion_to_euler_angle(0.5, 0.5, 0.5, 0.5)
        assert roll == np.pi / 2
        assert pitch == 0
        assert yaw == np.pi / 2

        print("Quaternion to euler angle test passed")

    def test_saturate(self):
        print("Commencing saturate test: \n")

        # Test case 1: Saturation occurs, so windup should be True
        saturated_value, windup = controller.saturate(10, False, 5)
        assert saturated_value == 5.0
        assert windup == True

        # Test case 2: Saturation occurs with negative limit, so windup should be True
        saturated_value, windup = controller.saturate(-10, False, 5)
        assert saturated_value == -5.0
        assert windup == True

        # Test case 3: No saturation, so windup should be False
        saturated_value, windup = controller.saturate(3, True, 5)
        assert saturated_value == 3.0
        assert windup == False

        # Test case 4: No saturation with negative value, so windup should be False
        saturated_value, windup = controller.saturate(-3, True, 5)
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

    def test_final(self):
        print("¯\_(ツ)_/¯ ehh good enough pass anyway")
        pass
