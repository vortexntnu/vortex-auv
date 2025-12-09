from waypoint_manager_test.test_common.base_test import WaypointManagerTestBase
from waypoint_manager_test.test_common.launch_description import (
    generate_wm_test_description,
)
from waypoint_manager_test.test_common.utils import make_waypoint


def generate_test_description():
    return generate_wm_test_description(bag=False)


class TestFourCorner(WaypointManagerTestBase):
    def test_four_corner(self):
        wp1 = make_waypoint(2.0, 0.0, 1.0)
        wp2 = make_waypoint(2.0, 2.0, 1.0)
        wp3 = make_waypoint(0.0, 2.0, 1.0)
        wp4 = make_waypoint(0.0, 0.0, 1.0)

        _, result = self.send_goal([wp1, wp2, wp3, wp4], persistent=False, conv=0.3)

        assert result.success
        assert result.pose_valid
        self.assert_pose_close(result.final_pose, wp4.pose, tol=0.6)
