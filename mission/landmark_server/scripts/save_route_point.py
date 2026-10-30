#!/usr/bin/env python3
"""Save where the vehicle is as a route point, for the test days.

Drive the vehicle to the spot, then:

    ros2 run landmark_server save_route_point.py home_1 torpedo_board

writes targets.home_1 = the vehicle's pose in torpedo_board's axes into the
routes file, and sets it on the running server if it already has it.
"""

import argparse
import math
import os
import re
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("name", help="target name, e.g. home_1")
    ap.add_argument("parent", help="frame it hangs on, e.g. torpedo_board")
    ap.add_argument("--frame", default="base_link", help="what to save")
    ap.add_argument("--drone", default="nautilus")
    ap.add_argument("--sim", action="store_true", help="routes_sim.yaml")
    ap.add_argument("--file", help="routes file (default: the package's)")
    ap.add_argument("--position-only", action="store_true", help="no yaw")
    args = ap.parse_args()
    path = args.file or os.path.join(
        get_package_share_directory("landmark_server"),
        "config",
        "routes_sim.yaml" if args.sim else "routes.yaml",
    )

    rclpy.init()
    node = Node("save_route_point")
    tf = Buffer()
    TransformListener(tf, node)
    prefix = args.drone + "/"
    t = None
    end = time.time() + 5.0
    while t is None and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            t = tf.lookup_transform(
                prefix + args.parent, prefix + args.frame, Time()
            ).transform
        except TransformException:
            pass
    if t is None:
        print(f"No transform from {args.parent} to {args.frame}")
        return 1

    q = t.rotation
    yaw = math.degrees(
        math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    )
    values = {
        "x": round(t.translation.x, 2),
        "y": round(t.translation.y, 2),
        "z": round(t.translation.z, 2),
    }
    if not args.position_only:
        values["yaw"] = round(yaw, 1)
    nums = ", ".join(f"{k}: {float(v)}" for k, v in values.items())
    line = f'      {args.name}: {{parents: ["{args.parent}"], {nums}}}\n'

    with open(path) as f:
        lines = f.readlines()
    key = re.compile(rf"^\s+{re.escape(args.name)}:\s")
    i = next((i for i, l in enumerate(lines) if key.match(l)), None)
    if i is None:
        lines.append(line)
    else:
        lines[i] = line
    with open(path, "w") as f:
        f.writelines(lines)
    print(f"{path}:\n{line}", end="")

    # Live too, if the server already has this target.
    client = node.create_client(
        SetParameters, f"/{args.drone}/landmark_server_node/set_parameters"
    )
    if not client.wait_for_service(timeout_sec=2.0):
        print("landmark_server is not running: it reads the file at start")
        return 0
    req = SetParameters.Request()
    k = f"targets.{args.name}."
    req.parameters = [Parameter(k + "parents", value=[args.parent]).to_parameter_msg()]
    req.parameters += [
        Parameter(k + n, value=float(v)).to_parameter_msg() for n, v in values.items()
    ]
    fut = client.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=3.0)
    ok = fut.result() and all(r.successful for r in fut.result().results)
    print("set on the server" if ok else "new target: restart landmark_server")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
