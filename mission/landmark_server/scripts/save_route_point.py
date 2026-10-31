#!/usr/bin/env python3
"""Save where the vehicle is as a route point, for the test days.

Drive the vehicle to the spot, then:

    ros2 run landmark_server save_route_point.py home_1 torpedo_board

writes targets.home_1 = the vehicle's pose in torpedo_board's axes into the
routes file, and sets it on the running server.
"""

import argparse
import math
import os
import re
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def routes_path(sim=False):
    return os.path.join(
        get_package_share_directory("landmark_server"),
        "config",
        "routes_sim.yaml" if sim else "routes.yaml",
    )


def write_point(path, name, parent, values):
    """Writes or replaces targets.<name> in the routes file. The line."""
    nums = ", ".join(f"{k}: {float(v)}" for k, v in values.items())
    line = f'      {name}: {{parents: ["{parent}"], {nums}}}\n'
    with open(path) as f:
        lines = f.readlines()
    # A file with no points yet is "    {}": the server does not take an
    # empty targets.
    lines = ["    targets:\n" if l.strip() == "{}" else l for l in lines]
    key = re.compile(rf"^\s+{re.escape(name)}:\s")
    i = next((i for i, l in enumerate(lines) if key.match(l)), None)
    if i is None:
        lines.append(line)
    else:
        lines[i] = line
    with open(path, "w") as f:
        f.writelines(lines)
    return line


def set_values(path, name, values):
    """Sets the numbers of targets.<name> in the file. The line, or None.

    Keeps its parents, other keys and comment.
    """
    with open(path) as f:
        lines = f.readlines()
    start = next((i for i, l in enumerate(lines) if l.strip() == "targets:"), None)
    if start is None:
        return None
    indent = len(lines[start]) - len(lines[start].lstrip())
    key = re.compile(rf"^(\s+{re.escape(name)}:\s*)(\{{.*?\}})(.*)$", re.S)
    for i in range(start + 1, len(lines)):
        line = lines[i]
        if line.strip() and len(line) - len(line.lstrip()) <= indent:
            break
        m = key.match(line)
        if not m:
            continue
        target = yaml.safe_load(m[2])
        for k, v in values.items():
            if k in target or v != 0:
                target[k] = float(v)
        lines[i] = m[1] + flow(target) + m[3]
        with open(path, "w") as f:
            f.writelines(lines)
        return lines[i]
    return None


def flow(target):
    def text(v):
        if isinstance(v, list):
            return "[" + ", ".join(f'"{s}"' for s in v) + "]"
        if isinstance(v, str):
            return f'"{v}"'
        return str(float(v))

    return "{" + ", ".join(f"{k}: {text(v)}" for k, v in target.items()) + "}"


def points_in(path):
    """(name, parents) of every target in the routes file."""
    out = []
    for line in open(path):
        m = re.match(r"^\s+(\w+): \{parents: \[([^\]]*)\]", line)
        if m:
            out.append((m[1], re.findall(r'"([^"]+)"', m[2])))
    return out


def set_request(name, parent, values):
    """Parent None leaves the parents as they are."""
    req = SetParameters.Request()
    k = f"targets.{name}."
    req.parameters = (
        [Parameter(k + "parents", value=[parent]).to_parameter_msg()] if parent else []
    )
    req.parameters += [
        Parameter(k + n, value=float(v)).to_parameter_msg() for n, v in values.items()
    ]
    return req


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
    path = args.file or routes_path(args.sim)

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
    line = write_point(path, args.name, args.parent, values)
    print(f"{path}:\n{line}", end="")

    # Live too.
    client = node.create_client(
        SetParameters, f"/{args.drone}/landmark_server_node/set_parameters"
    )
    if not client.wait_for_service(timeout_sec=2.0):
        print("landmark_server is not running: it reads the file at start")
        return 0
    fut = client.call_async(set_request(args.name, args.parent, values))
    rclpy.spin_until_future_complete(node, fut, timeout_sec=3.0)
    ok = fut.result() and all(r.successful for r in fut.result().results)
    print("set on the server" if ok else "not set on the server")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
