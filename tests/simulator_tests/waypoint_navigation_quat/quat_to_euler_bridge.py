#!/usr/bin/env python3
"""Bridge node that converts ReferenceFilterQuat to ReferenceFilter (Euler)."""

import rclpy
from rclpy.node import Node
from vortex_msgs.msg import ReferenceFilter, ReferenceFilterQuat
from vortex_utils.python_utils import quat_to_euler


class QuatToEulerBridge(Node):
    def __init__(self):
        super().__init__('quat_to_euler_bridge')

        qos_profile = rclpy.qos.QoSProfile(
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
        )

        self.sub_ = self.create_subscription(
            ReferenceFilterQuat,
            '/nautilus/guidance/dp_quat',
            self.callback,
            qos_profile,
        )

        self.pub_ = self.create_publisher(
            ReferenceFilter,
            '/nautilus/guidance/dp',
            qos_profile,
        )

        self.get_logger().info(
            'Bridge: /nautilus/guidance/dp_quat -> /nautilus/guidance/dp'
        )

    def callback(self, msg: ReferenceFilterQuat):
        euler = quat_to_euler(x=msg.qx, y=msg.qy, z=msg.qz, w=msg.qw)

        out = ReferenceFilter()
        out.header = msg.header

        out.x = msg.x
        out.y = msg.y
        out.z = msg.z
        out.roll = float(euler[0])
        out.pitch = float(euler[1])
        out.yaw = float(euler[2])

        out.x_dot = msg.x_dot
        out.y_dot = msg.y_dot
        out.z_dot = msg.z_dot

        # For this test i dont care about the
        # euler rate vs NED angular velocity distinction
        out.roll_dot = msg.roll_dot
        out.pitch_dot = msg.pitch_dot
        out.yaw_dot = msg.yaw_dot

        self.pub_.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = QuatToEulerBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
