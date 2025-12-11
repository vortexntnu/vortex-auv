from vortex_msgs.action import WaypointManager
from vortex_msgs.srv import SendWaypoints
from waypoint_manager_test.test_common.base_test import WaypointManagerTestBase
from waypoint_manager_test.test_common.launch_description import (
    generate_wm_test_description,
)
from waypoint_manager_test.test_common.utils import make_waypoint


def generate_test_description():
    return generate_wm_test_description()


class TestServiceFailure(WaypointManagerTestBase):
    def test_service_no_active_action(self):
        wp = make_waypoint(2.0, 0.0, 1.0)

        req = SendWaypoints.Request()
        req.waypoints = [wp]
        req.overwrite_prior_waypoints = False
        req.take_priority = False

        fut = self.wp_add_client.call_async(req)
        resp = self.spin_until_done(fut, timeout=10)

        assert not resp.success, "Service call succeeded despite no active action"

    def test_service_non_persistent_action(self):
        wp1 = make_waypoint(5.0, 0.0, 0.5)

        goal = WaypointManager.Goal()
        goal.waypoints = [wp1]
        goal.persistent = False
        goal.convergence_threshold = 0.3

        send_fut = self.action_client.send_goal_async(goal)
        handle = self.spin_until_done(send_fut, timeout=120)
        assert handle.accepted, "Goal was rejected"

        wp2 = make_waypoint(2.0, 0.0, 0.5)

        req = SendWaypoints.Request()
        req.waypoints = [wp2]
        req.overwrite_prior_waypoints = False
        req.take_priority = False

        fut = self.wp_add_client.call_async(req)
        resp = self.spin_until_done(fut, timeout=10)

        assert not resp.success, "Service call succeeded despite non-persistent action"
