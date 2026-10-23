#!/usr/bin/env python3
import argparse
import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from vortex_msgs.action import GuidanceWaypoint
from vortex_msgs.msg import OperationMode, WaypointMode
from vortex_msgs.srv import SetKillswitch, SetOperationMode


def euler_to_quaternion(roll, pitch, yaw):
    """ZYX convention (Fossen), angles in radians."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qw, qx, qy, qz


class WaypointSender(Node):
    def __init__(self):
        super().__init__('waypoint_sender')
        self._client = ActionClient(
            self, GuidanceWaypoint, '/nautilus/reference_filter'
        )
        self._operation_mode_client = self.create_client(
            SetOperationMode, '/nautilus/set_operation_mode'
        )
        self._killswitch_client = self.create_client(
            SetKillswitch, '/nautilus/set_killswitch'
        )

    def wait_for(self, is_ready, what, timeout_s):
        self.get_logger().info(f'Waiting for {what}...')
        deadline = self.get_clock().now().nanoseconds + int(timeout_s * 1e9)
        while not is_ready():
            if self.get_clock().now().nanoseconds > deadline:
                self.get_logger().error(
                    f'{what} not found. Is the sim running with the quat '
                    'controller (auv_setup dp_quat.launch.py)?'
                )
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return True

    def arm(self, timeout_s=120.0):
        """Switch to autonomous mode and turn the killswitch off."""
        if not (
            self.wait_for(
                lambda: self._operation_mode_client.service_is_ready(),
                'service /nautilus/set_operation_mode',
                timeout_s,
            )
            and self.wait_for(
                lambda: self._killswitch_client.service_is_ready(),
                'service /nautilus/set_killswitch',
                timeout_s,
            )
        ):
            return False

        mode_request = SetOperationMode.Request()
        mode_request.requested_operation_mode.operation_mode = (
            OperationMode.AUTONOMOUS
        )
        future = self._operation_mode_client.call_async(mode_request)
        rclpy.spin_until_future_complete(self, future)
        if not future.result().success:
            self.get_logger().error('Failed to set autonomous mode')
            return False

        killswitch_request = SetKillswitch.Request()
        killswitch_request.killswitch_on = False
        future = self._killswitch_client.call_async(killswitch_request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if not response.success or response.killswitch_status:
            self.get_logger().error('Failed to turn the killswitch off')
            return False

        self.get_logger().info('Autonomous mode, killswitch off')
        return True

    def send_square(self, side, depth, loops, threshold=0.2):
        """Fly the corners of a square forever, or for a number of loops."""
        corners = [(0.0, 0.0), (side, 0.0), (side, side), (0.0, side)]
        loop = 0
        while loops == 0 or loop < loops:
            loop += 1
            for corner, (x, y) in enumerate(corners, start=1):
                self.get_logger().info(
                    f'Loop {loop}, corner {corner}/{len(corners)}'
                )
                if not self.send_goal(
                    x, y, depth, 0.0, 0.0, 0.0, threshold=threshold, wait_s=60.0
                ):
                    return False
        return True

    def send_figure_eight(self, size, depth, loops, points=16, threshold=0.3):
        """Fly a figure 8 forever, or for a number of loops.

        The path is x = size * sin(t), y = size / 2 * sin(2t), a lemniscate of
        Gerono that starts at the origin and is sampled at evenly spaced t. Each
        sample is a waypoint, so the looser threshold keeps the vehicle moving.
        """
        loop = 0
        while loops == 0 or loop < loops:
            loop += 1
            for index in range(1, points + 1):
                t = 2.0 * math.pi * index / points
                x = size * math.sin(t)
                y = 0.5 * size * math.sin(2.0 * t)
                self.get_logger().info(f'Loop {loop}, point {index}/{points}')
                if not self.send_goal(
                    x, y, depth, 0.0, 0.0, 0.0, threshold=threshold, wait_s=60.0
                ):
                    return False
        return True

    def send_goal(self, x, y, z, roll_deg, pitch_deg, yaw_deg,
                  mode=WaypointMode.FULL_POSE, threshold=0.1, wait_s=5.0):
        if not self.wait_for(
            lambda: self._client.server_is_ready(),
            'action server /nautilus/reference_filter',
            wait_s,
        ):
            return False

        goal = GuidanceWaypoint.Goal()

        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        yaw = math.radians(yaw_deg)
        qw, qx, qy, qz = euler_to_quaternion(roll, pitch, yaw)

        goal.waypoint.pose.position.x = x
        goal.waypoint.pose.position.y = y
        goal.waypoint.pose.position.z = z
        goal.waypoint.pose.orientation.w = qw
        goal.waypoint.pose.orientation.x = qx
        goal.waypoint.pose.orientation.y = qy
        goal.waypoint.pose.orientation.z = qz
        goal.waypoint.waypoint_mode.mode = mode
        goal.convergence_threshold = threshold

        self.get_logger().info(
            f'Sending: pos=({x}, {y}, {z}), '
            f'rpy=({roll_deg:.1f}, {pitch_deg:.1f}, {yaw_deg:.1f}) deg, '
            f'mode={mode}, threshold={threshold}'
        )

        future = self._client.send_goal_async(
            goal, feedback_callback=self.feedback_cb
        )
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return False

        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        self.get_logger().info(f'Success: {result.success}')
        return result.success

    def feedback_cb(self, feedback_msg):
        ref = feedback_msg.feedback.reference
        self.get_logger().info(
            f'  ref: pos=({ref.x:.2f}, {ref.y:.2f}, {ref.z:.2f}) '
            f'rpy=({math.degrees(ref.roll):.1f}, {math.degrees(ref.pitch):.1f}, {math.degrees(ref.yaw):.1f}) deg'
        )


def parse_args():
    parser = argparse.ArgumentParser(
        description='Send a waypoint to the quat DP reference filter.'
    )
    parser.add_argument('x', type=float, nargs='?', help='X position [m]')
    parser.add_argument('y', type=float, nargs='?', help='Y position [m]')
    parser.add_argument('z', type=float, nargs='?', help='Z position [m]')
    parser.add_argument('roll', type=float, nargs='?', default=0.0, help='Roll [deg] (default: 0)')
    parser.add_argument('pitch', type=float, nargs='?', default=0.0, help='Pitch [deg] (default: 0)')
    parser.add_argument('yaw', type=float, nargs='?', default=0.0, help='Yaw [deg] (default: 0)')
    parser.add_argument('--threshold', type=float, default=0.1, help='Convergence threshold (default: 0.1)')
    parser.add_argument(
        '--mode', type=int, default=WaypointMode.FULL_POSE,
        choices=[WaypointMode.FULL_POSE, WaypointMode.ONLY_POSITION,
                 WaypointMode.FORWARD_HEADING, WaypointMode.ONLY_ORIENTATION],
        help='Waypoint mode: 0=FULL_POSE, 1=ONLY_POSITION, 2=FORWARD_HEADING, 3=ONLY_ORIENTATION (default: 0)'
    )
    parser.add_argument(
        '--square', type=float, metavar='SIDE',
        help='Fly a square with this side length [m] forever, instead of a single waypoint'
    )
    parser.add_argument(
        '--figure8', type=float, metavar='SIZE',
        help='Fly a figure 8 with this half width [m] forever, instead of a single waypoint'
    )
    parser.add_argument('--depth', type=float, default=2.0, help='Depth of the pattern [m] (default: 2)')
    parser.add_argument('--loops', type=int, default=0, help='Number of pattern loops, 0 is forever (default: 0)')
    parser.add_argument(
        '--arm', action='store_true',
        help='Switch to autonomous mode and turn the killswitch off first (always done for --square and --figure8)'
    )
    args = parser.parse_args()
    if args.square is not None and args.figure8 is not None:
        parser.error('--square and --figure8 cannot be combined')
    if (
        args.square is None
        and args.figure8 is None
        and None in (args.x, args.y, args.z)
    ):
        parser.error('x, y and z are required unless --square or --figure8 is given')
    return args


def main():
    args = parse_args()
    rclpy.init()
    sender = WaypointSender()

    try:
        if args.square is not None or args.figure8 is not None or args.arm:
            if not sender.arm():
                return

        if args.square is not None:
            sender.send_square(args.square, args.depth, args.loops)
        elif args.figure8 is not None:
            sender.send_figure_eight(args.figure8, args.depth, args.loops)
        else:
            sender.send_goal(
                args.x, args.y, args.z,
                args.roll, args.pitch, args.yaw,
                mode=args.mode,
                threshold=args.threshold,
            )
    except KeyboardInterrupt:
        pass
    finally:
        sender.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
