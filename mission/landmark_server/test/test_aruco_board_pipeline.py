#!/usr/bin/env python3
"""
Integration test: LandmarkPolling → LandmarkConvergence for ArUco board sonar.

Target landmark:
  type    = 2  (ARUCO_BOARD)
  subtype = 2  (ARUCO_BOARD_SONAR)

Usage:
  1. Launch the simulator and aruco detection separately.
  2. Launch the landmark server:
       ros2 launch landmark_server landmark_server.launch.py
  3. Launch the reference filter node.
  4. Run this script:
       python3 test/test_aruco_board_pipeline.py
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from action_msgs.msg import GoalStatus
from vortex_msgs.action import LandmarkPolling, LandmarkConvergence
from vortex_msgs.msg import LandmarkType, LandmarkSubtype

# ─── Target landmark ──────────────────────────────────────────────────────────
TARGET_TYPE    = LandmarkType.ARUCO_BOARD   # 2
TARGET_SUBTYPE = LandmarkSubtype.ARUCO_BOARD_CAMERA # 2

# ─── Convergence parameters ───────────────────────────────────────────────────
# Stand 1.5 m directly in front of the board (along its +X axis)
CONVERGENCE_OFFSET_X   = 0.0   # metres
CONVERGENCE_OFFSET_Y   = 0.0
CONVERGENCE_OFFSET_Z   = 0.0
CONVERGENCE_THRESHOLD  = 0.1   # metres – how cl3ose is "arrived"
DEAD_RECKONING_OFFSET  = 0.8   # metres – switch to DR when this close
TRACK_LOSS_TIMEOUT     = 5.0   # seconds – abort if track gone this long

# ─── Timeouts ─────────────────────────────────────────────────────────────────
SERVER_WAIT_TIMEOUT_S  = 10.0  # wait for action servers
POLLING_TIMEOUT_S      = 60.0  # max time to wait for a landmark to appear

# ─── Action server names ──────────────────────────────────────────────────────
# These must match what landmark_server_node advertises (orca namespace).
POLLING_ACTION    = "/orca/landmark_polling"
CONVERGENCE_ACTION = "/orca/landmark_convergence"


class PipelineTester(Node):
    def __init__(self):
        super().__init__("aruco_board_pipeline_tester")

        self._polling_client    = ActionClient(self, LandmarkPolling,    POLLING_ACTION)
        self._convergence_client = ActionClient(self, LandmarkConvergence, CONVERGENCE_ACTION)

        self.get_logger().info(
            f"Waiting for action servers "
            f"(timeout={SERVER_WAIT_TIMEOUT_S:.0f}s)..."
        )
        if not self._polling_client.wait_for_server(
                timeout_sec=SERVER_WAIT_TIMEOUT_S):
            self.get_logger().fatal(
                f"Polling action server '{POLLING_ACTION}' not available")
            raise RuntimeError("Polling server not available")

        if not self._convergence_client.wait_for_server(
                timeout_sec=SERVER_WAIT_TIMEOUT_S):
            self.get_logger().fatal(
                f"Convergence action server '{CONVERGENCE_ACTION}' not available")
            raise RuntimeError("Convergence server not available")

        self.get_logger().info("Both action servers are ready.")

    # ─────────────────────────────────────────────────────────────────────────
    # Phase 1 – Polling
    # ─────────────────────────────────────────────────────────────────────────
    def run_polling(self) -> bool:
        """
        Send a LandmarkPolling goal for (type=2, subtype=2).
        Blocks until a confirmed landmark is found or an error occurs.
        Returns True on success.
        """
        self.get_logger().info(
            f"─── Phase 1: LandmarkPolling "
            f"(type={TARGET_TYPE}, subtype={TARGET_SUBTYPE}) ───"
        )

        goal = LandmarkPolling.Goal()
        goal.type.value    = TARGET_TYPE
        goal.subtype.value = TARGET_SUBTYPE

        send_future = self._polling_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(
            self, send_future, timeout_sec=SERVER_WAIT_TIMEOUT_S)

        if not send_future.done():
            self.get_logger().error("send_goal timed out for polling")
            return False

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error(
                "Polling goal REJECTED by server. "
                "Is the landmark server running?"
            )
            return False

        self.get_logger().info(
            f"Polling goal ACCEPTED – scanning for ArUco board sonar "
            f"(timeout={POLLING_TIMEOUT_S:.0f}s)..."
        )

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=POLLING_TIMEOUT_S)

        if not result_future.done():
            self.get_logger().error(
                f"Polling timed out after {POLLING_TIMEOUT_S:.0f}s – "
                f"no landmark found. "
                f"Check aruco detection and TF."
            )
            goal_handle.cancel_goal_async()
            return False

        wrapped = result_future.result()
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f"Polling did not succeed (status={wrapped.status})"
            )
            return False

        landmarks = wrapped.result.landmarks.landmarks
        if not landmarks:
            self.get_logger().error(
                "Polling succeeded but returned an empty landmark list. "
                "This should not happen – check tracks_to_landmark_msgs()."
            )
            return False

        self.get_logger().info(
            f"✅ Polling SUCCEEDED – found {len(landmarks)} landmark(s):"
        )
        for lm in landmarks:
            p = lm.pose.pose.position
            self.get_logger().info(
                f"   id={lm.id}  type={lm.type.value}  subtype={lm.subtype.value}"
                f"  pos=({p.x:.3f}, {p.y:.3f}, {p.z:.3f})"
            )

        return True

    # ─────────────────────────────────────────────────────────────────────────
    # Phase 2 – Convergence
    # ─────────────────────────────────────────────────────────────────────────
    def run_convergence(self) -> bool:
        """
        Send a LandmarkConvergence goal for (type=2, subtype=2).
        Uses the landmark found during polling.
        Returns True on success.
        """
        self.get_logger().info(
            f"─── Phase 2: LandmarkConvergence "
            f"(type={TARGET_TYPE}, subtype={TARGET_SUBTYPE}) ───"
        )

        goal = LandmarkConvergence.Goal()
        goal.type.value    = TARGET_TYPE
        goal.subtype.value = TARGET_SUBTYPE

        # Position offset relative to the landmark's frame
        goal.convergence_offset.position.x    = CONVERGENCE_OFFSET_X
        goal.convergence_offset.position.y    = CONVERGENCE_OFFSET_Y
        goal.convergence_offset.position.z    = CONVERGENCE_OFFSET_Z
        goal.convergence_offset.orientation.w = 1.0  # no rotation offset
        goal.convergence_offset.orientation.x = 0.0
        goal.convergence_offset.orientation.y = 0.0
        goal.convergence_offset.orientation.z = 0.0

        goal.convergence_threshold  = CONVERGENCE_THRESHOLD
        goal.dead_reckoning_offset  = DEAD_RECKONING_OFFSET
        goal.track_loss_timeout_sec     = TRACK_LOSS_TIMEOUT

        send_future = self._convergence_client.send_goal_async(
            goal, feedback_callback=self._convergence_feedback
        )
        rclpy.spin_until_future_complete(
            self, send_future, timeout_sec=SERVER_WAIT_TIMEOUT_S)

        if not send_future.done():
            self.get_logger().error("send_goal timed out for convergence")
            return False

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error(
                "Convergence goal REJECTED. "
                "Possible reasons:\n"
                "  – No confirmed track yet (track not confirmed after polling)\n"
                "  – ReferenceFilter server not running\n"
                "  – Invalid parameters"
            )
            return False

        self.get_logger().info(
            f"Convergence goal ACCEPTED – navigating to landmark "
            f"(offset=+{CONVERGENCE_OFFSET_X}m, "
            f"threshold={CONVERGENCE_THRESHOLD}m, "
            f"dr_offset={DEAD_RECKONING_OFFSET}m, "
            f"loss_timeout={TRACK_LOSS_TIMEOUT}s)..."
        )

        result_future = goal_handle.get_result_async()
        # No hard timeout here – let convergence run until done or track loss
        rclpy.spin_until_future_complete(self, result_future)

        wrapped = result_future.result()

        if wrapped.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("✅ Convergence SUCCEEDED")
            return True
        elif wrapped.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("⚠️  Convergence CANCELED")
            return False
        else:
            self.get_logger().error(
                f"❌ Convergence ABORTED (status={wrapped.status}). "
                f"Possible reasons:\n"
                f"  – Track lost for > {TRACK_LOSS_TIMEOUT}s\n"
                f"  – ReferenceFilter aborted\n"
                f"  – Invalid computed target pose"
            )
            return False

    def _convergence_feedback(self, feedback_msg):
        # ReferenceFilterWaypoint feedback is forwarded through the server
        self.get_logger().info(
            "Convergence: feedback received", throttle_duration_sec=2.0)


# ─── Entry point ──────────────────────────────────────────────────────────────
def main():
    rclpy.init()

    executor = SingleThreadedExecutor()
    try:
        tester = PipelineTester()
    except RuntimeError as e:
        print(f"[FATAL] {e}")
        rclpy.shutdown()
        sys.exit(1)

    executor.add_node(tester)

    print("\n" + "=" * 60)
    print("  ArUco Board Sonar – Landmark Server Integration Test")
    print(f"  target: type={TARGET_TYPE} (ARUCO_BOARD)  "
          f"subtype={TARGET_SUBTYPE} (ARUCO_BOARD_SONAR)")
    print("=" * 60 + "\n")

    # ── Phase 1: Polling ──────────────────────────────────────────────────────
    polling_ok = tester.run_polling()
    if not polling_ok:
        tester.get_logger().fatal("Polling phase failed – aborting test")
        executor.shutdown()
        rclpy.shutdown()
        sys.exit(1)

    # ── Phase 2: Convergence ──────────────────────────────────────────────────
    convergence_ok = tester.run_convergence()

    print("\n" + "=" * 60)
    if polling_ok and convergence_ok:
        print("  RESULT: ✅  Full pipeline PASSED")
        exit_code = 0
    elif polling_ok:
        print("  RESULT: ⚠️   Polling OK – Convergence FAILED")
        exit_code = 1
    else:
        print("  RESULT: ❌  FAILED")
        exit_code = 1
    print("=" * 60 + "\n")

    executor.shutdown()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
