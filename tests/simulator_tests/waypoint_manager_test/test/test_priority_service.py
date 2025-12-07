import time

from vortex_msgs.action import WaypointManager
from vortex_msgs.srv import WaypointAddition

from waypoint_manager_test.test_common.base_test import WaypointManagerTestBase
from waypoint_manager_test.test_common.launch_description import (
    generate_wm_test_description,
)
from waypoint_manager_test.test_common.utils import make_waypoint


def generate_test_description():
    return generate_wm_test_description()


class TestPriority(WaypointManagerTestBase):
    def test_priority_service_behavior(self):
        wp1 = make_waypoint(4.0, 0.0, 0.0)
        wp2 = make_waypoint(-4.0, 0.0, 0.0)

        goal = WaypointManager.Goal()
        goal.waypoints = []
        goal.persistent = True
        goal.convergence_threshold = 0.3

        send_fut = self.action_client.send_goal_async(goal)
        handle = self.spin_until_done(send_fut, timeout=120)
        assert handle.accepted, "Goal was rejected"

        req = WaypointAddition.Request()
        req.waypoints = [wp1]
        req.overwrite = False
        req.priority = True

        fut = self.wp_add_client.call_async(req)
        resp = self.spin_until_done(fut, timeout=10)

        assert resp.success, "Setting priority=True failed"

        req2 = WaypointAddition.Request()
        req2.waypoints = [wp2]
        req2.overwrite = False
        req2.priority = False

        fut2 = self.wp_add_client.call_async(req2)
        resp2 = self.spin_until_done(fut2, timeout=10)

        assert not resp2.success, (
            "Service accepted a priority=False request while in priority mode"
        )

        # Spin for 10 seconds to ensure the vehicle is moving toward wp1
        start_time = time.time()
        while time.time() - start_time < 20:
            self.executor.spin_once(timeout_sec=0.1)

        cancel_fut = self.action_client._cancel_goal_async(handle)
        cancel_result = self.spin_until_done(cancel_fut, timeout=10)

        assert cancel_result.goals_canceling, "Cancel request was rejected"

        result_fut = handle.get_result_async()
        wrapped = self.spin_until_done(result_fut, timeout=20)
        final_result = wrapped.result

        x = final_result.final_pose.position.x
        assert x > 0.0, "Vehicle did not make progress toward the priority waypoint"
