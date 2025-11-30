from waypoint_manager_test.test_common.launch_setup import generate_wm_test_description
from waypoint_manager_test.test_common.base_test import WaypointManagerTestBase
from waypoint_manager_test.test_common.utilities import make_waypoint

import launch_testing

def generate_test_description():
    return generate_wm_test_description()

class TestSingleWaypoint(WaypointManagerTestBase):

    def test_single_wp(self):
        wp = make_waypoint(2.0, 0.0, 1.0)

        _, result = self.send_goal([wp], persistent=False, conv=0.3)

        self.assertTrue(result.success)
        self.assertTrue(result.pose_valid)
        self.assert_pose_close(result.final_pose, wp.pose, tol=0.6)
