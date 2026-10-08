#!/usr/bin/env python3
"""Edit the prior map on a top-down view and send it to landmark_slam.

Usage:
    ros2 run landmark_slam prior_map_gui.py --ros-args -r __ns:=/nautilus

Gets the prior map from landmark_slam (its prior_map parameter: the file's
text), with the live map (grey) and the vehicle (black) on top. Place the
start where the vehicle is put in the water and the objects where the course
drawing has them, relative to it. After a practice run the live map shows
where the objects really are (crosses in the class colour, with how often
each was seen: a real object is seen far more often than a phantom): drag
the entries there. Send sets prior_map: landmark_slam checks it, saves
it on the vehicle and uses it from the next anchoring. Anchor here starts a
new run (mission/wipe): the map is rebuilt with the vehicle's pose as the
start. Anchor with the vehicle at the start facing the course, then turn it
for the coin flip.

    left click        new entry of the chosen class
    left drag         move an entry or the start
    right click       remove an entry
    wheel, middle drag  zoom, pan
"""

import math
import os
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty
from tf2_ros import Buffer, TransformListener
from vortex_msgs.msg import LandmarkTrackArray

CONFIG_DIR = os.path.join(get_package_share_directory("landmark_slam"), "config")
PICK_PX = 10
COLORS = [
    "#d62728",
    "#1f77b4",
    "#2ca02c",
    "#ff7f0e",
    "#9467bd",
    "#8c564b",
    "#e377c2",
    "#17becf",
    "#bcbd22",
]


class PriorMap:
    """The file contents: start pose (yaw in rad) and the entries."""

    def __init__(self, data=None):
        data = data or {}
        self.start = {
            "x": 0.0,
            "y": 0.0,
            "yaw": 0.0,
            "sigma_xy": 0.5,
            "sigma_yaw": 0.3,
            **(data.get("initial_pose") or {}),
        }
        self.entries = [dict(e) for e in data.get("landmarks") or []]

    def dump(self):
        def flow(d):
            return (
                "{"
                + ", ".join(
                    f"{k}: {round(v, 4) if isinstance(v, float) else v}"
                    for k, v in d.items()
                )
                + "}"
            )

        lines = [
            f"# Written by prior_map_gui.py, {time.strftime('%Y-%m-%d %H:%M')}",
            f"initial_pose: {flow(self.start)}",
            "landmarks:",
        ]
        lines += [f"  - {flow(e)}" for e in self.entries]
        return "\n".join(lines) + "\n"


class Ros(Node):
    def __init__(self):
        super().__init__("prior_map_gui")
        ns = self.get_namespace().strip("/")
        prefix = ns + "/" if ns else ""
        self.slam = self.declare_parameter("slam_node", "landmark_slam_node").value
        self.classes_file = self.declare_parameter(
            "classes_file", os.path.join(CONFIG_DIR, "landmark_classes.yaml")
        ).value
        self.map_frame = self.declare_parameter("map_frame", prefix + "map").value
        self.base_frame = self.declare_parameter(
            "base_frame", prefix + "base_link"
        ).value
        self.get_cli = self.create_client(GetParameters, f"{self.slam}/get_parameters")
        self.set_cli = self.create_client(SetParameters, f"{self.slam}/set_parameters")
        self.tracks = []
        self.wipe_pub = self.create_publisher(
            Empty,
            "mission/wipe",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
        )
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.create_subscription(
            LandmarkTrackArray,
            "landmark_slam/landmarks",
            lambda m: setattr(self, "tracks", m.landmark_tracks),
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )

    def call(self, client, request, timeout=3.0):
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        future = client.call_async(request)
        end = time.time() + timeout
        while not future.done() and time.time() < end:
            time.sleep(0.05)
        return future.result()

    def prior_map(self):
        res = self.call(self.get_cli, GetParameters.Request(names=["prior_map"]))
        return res.values[0].string_value if res and res.values else None

    def set_prior_map(self, text):
        value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=text)
        res = self.call(
            self.set_cli,
            SetParameters.Request(
                parameters=[Parameter(name="prior_map", value=value)]
            ),
        )
        if res is None:
            return False, f"{self.slam} did not answer"
        return res.results[0].successful, res.results[0].reason

    def vehicle(self):
        try:
            t = self.tf.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time()
            ).transform
        except Exception:
            return None
        q = t.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.translation.x, t.translation.y, yaw


class Gui:
    def __init__(self, root, ros):
        self.root, self.ros = root, ros
        classes = yaml.safe_load(open(ros.classes_file))
        self.classes = sorted(classes)
        self.class_of = {(c["type"], c["subtype"]): n for n, c in classes.items()}
        self.color = {c: COLORS[i % len(COLORS)] for i, c in enumerate(self.classes)}
        self.prior = PriorMap()
        self.scale, self.origin = 30.0, (100.0, 400.0)  # px per m, screen of (0, 0)
        self.drag = None  # ("entry", i) | ("start",) | ("pan", x, y)
        self.loading = False

        root.title("Prior map")
        side = ttk.Frame(root, padding=8)
        side.pack(side=tk.RIGHT, fill=tk.Y)
        self.canvas = tk.Canvas(root, width=900, height=650, background="white")
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        ttk.Label(side, text="New entry").pack(anchor="w")
        self.cls = tk.StringVar(value=self.classes[0])
        ttk.Combobox(
            side, textvariable=self.cls, values=self.classes, state="readonly", width=24
        ).pack(anchor="w")
        self.sigma = self.field(side, "sigma_xy [m]", 1.0)
        ttk.Separator(side).pack(fill=tk.X, pady=6)
        ttk.Label(side, text="Start (where the vehicle is put in)").pack(anchor="w")
        self.start_vars = {
            k: self.field(side, label, 0.0)
            for k, label in [
                ("x", "x [m]"),
                ("y", "y [m]"),
                ("yaw", "yaw [deg]"),
                ("sigma_xy", "sigma_xy [m]"),
                ("sigma_yaw", "sigma_yaw [deg]"),
            ]
        }
        for v in self.start_vars.values():
            v.trace_add("write", lambda *_: self.read_start())
        ttk.Separator(side).pack(fill=tk.X, pady=6)
        for text, cmd in [
            ("Send to landmark_slam", self.send),
            ("Anchor here (new run)", self.anchor),
            ("Get from landmark_slam", self.get),
            ("Open file...", self.open_file),
            ("Save as...", self.save_as),
            ("Fit view", self.fit),
        ]:
            ttk.Button(side, text=text, command=cmd).pack(fill=tk.X, pady=2)
        self.status = tk.StringVar()
        ttk.Label(side, textvariable=self.status, wraplength=220, justify=tk.LEFT).pack(
            anchor="w", pady=8
        )

        c = self.canvas
        c.bind("<ButtonPress-1>", self.press)
        c.bind("<B1-Motion>", self.motion)
        c.bind("<ButtonRelease-1>", lambda e: setattr(self, "drag", None))
        c.bind("<ButtonPress-3>", self.remove)
        c.bind("<ButtonPress-2>", lambda e: setattr(self, "drag", ("pan", e.x, e.y)))
        c.bind("<B2-Motion>", self.motion)
        c.bind("<MouseWheel>", lambda e: self.zoom(e, e.delta > 0))
        c.bind("<Button-4>", lambda e: self.zoom(e, True))
        c.bind("<Button-5>", lambda e: self.zoom(e, False))
        c.bind("<Configure>", lambda e: self.draw())

        self.get()
        self.tick()

    def field(self, parent, label, value):
        row = ttk.Frame(parent)
        row.pack(fill=tk.X)
        ttk.Label(row, text=label, width=16).pack(side=tk.LEFT)
        var = tk.StringVar(value=f"{value:g}")
        ttk.Entry(row, textvariable=var, width=8).pack(side=tk.LEFT)
        return var

    # map <-> screen (x right, y up)
    def to_px(self, x, y):
        return self.origin[0] + x * self.scale, self.origin[1] - y * self.scale

    def to_m(self, px, py):
        return (px - self.origin[0]) / self.scale, (self.origin[1] - py) / self.scale

    def load(self, data, source):
        self.prior = PriorMap(data)
        s = dict(self.prior.start)
        self.loading = True  # the fields are set one by one
        for k, v in self.start_vars.items():
            deg = k in ("yaw", "sigma_yaw")
            v.set(f"{math.degrees(s[k]) if deg else s[k]:.4g}")
        self.loading = False
        self.status.set(f"{len(self.prior.entries)} entries from {source}")
        self.fit()

    def read_start(self):
        if self.loading:
            return
        try:
            vals = {k: float(v.get()) for k, v in self.start_vars.items()}
        except ValueError:
            return
        vals["yaw"] = math.radians(vals["yaw"])
        vals["sigma_yaw"] = math.radians(vals["sigma_yaw"])
        self.prior.start.update(vals)
        self.draw()

    def show_start(self):
        s = self.prior.start
        self.loading = True
        self.start_vars["x"].set(f"{s['x']:.2f}")
        self.start_vars["y"].set(f"{s['y']:.2f}")
        self.loading = False

    def get(self):
        text = self.ros.prior_map()
        if text is None:
            self.status.set(f"{self.ros.slam} did not answer: empty map")
            self.load({}, "nothing")
            return
        self.load(yaml.safe_load(text), self.ros.slam)

    def open_file(self):
        path = filedialog.askopenfilename(
            initialdir=CONFIG_DIR, filetypes=[("YAML", "*.yaml")]
        )
        if path:
            self.load(yaml.safe_load(open(path)), path)

    def save_as(self):
        path = filedialog.asksaveasfilename(
            defaultextension=".yaml", initialfile="prior_map.yaml"
        )
        if path:
            open(path, "w").write(self.prior.dump())
            self.status.set(f"saved {path}")

    def anchor(self):
        if not messagebox.askyesno(
            "Anchor",
            "Start a new run here? The map is rebuilt with the vehicle's pose as "
            "the start, and waypoint_manager's goals stop. The vehicle should be "
            "at the start, facing the course.",
        ):
            return
        self.ros.wipe_pub.publish(Empty())
        self.status.set("anchored: turn the vehicle for the coin flip now")

    def send(self):
        ok, reason = self.ros.set_prior_map(self.prior.dump())
        self.status.set(
            f"sent {len(self.prior.entries)} entries, used from the next mission start"
            if ok
            else f"rejected: {reason}"
        )

    def pick(self, px, py):
        sx, sy = self.to_px(self.prior.start["x"], self.prior.start["y"])
        if math.hypot(px - sx, py - sy) < PICK_PX:
            return ("start",)
        for i, e in enumerate(self.prior.entries):
            ex, ey = self.to_px(e["x"], e["y"])
            if math.hypot(px - ex, py - ey) < PICK_PX:
                return ("entry", i)
        return None

    def press(self, ev):
        self.drag = self.pick(ev.x, ev.y)
        if self.drag is None:
            x, y = self.to_m(ev.x, ev.y)
            try:
                sigma = float(self.sigma.get())
            except ValueError:
                sigma = 1.0
            self.prior.entries.append(
                {
                    "class": self.cls.get(),
                    "x": round(x, 2),
                    "y": round(y, 2),
                    "sigma_xy": sigma,
                }
            )
            self.drag = ("entry", len(self.prior.entries) - 1)
        self.draw()

    def motion(self, ev):
        if not self.drag:
            return
        if self.drag[0] == "pan":
            _, x0, y0 = self.drag
            self.origin = (self.origin[0] + ev.x - x0, self.origin[1] + ev.y - y0)
            self.drag = ("pan", ev.x, ev.y)
        else:
            x, y = (round(v, 2) for v in self.to_m(ev.x, ev.y))
            target = (
                self.prior.start
                if self.drag[0] == "start"
                else self.prior.entries[self.drag[1]]
            )
            target["x"], target["y"] = x, y
            if self.drag[0] == "start":
                self.show_start()
        self.draw()

    def remove(self, ev):
        hit = self.pick(ev.x, ev.y)
        if hit and hit[0] == "entry":
            del self.prior.entries[hit[1]]
            self.draw()

    def zoom(self, ev, zoom_in):
        f = 1.2 if zoom_in else 1 / 1.2
        self.origin = (
            ev.x + (self.origin[0] - ev.x) * f,
            ev.y + (self.origin[1] - ev.y) * f,
        )
        self.scale *= f
        self.draw()

    def fit(self):
        pts = [(e["x"], e["y"]) for e in self.prior.entries]
        pts.append((self.prior.start["x"], self.prior.start["y"]))
        xs, ys = [p[0] for p in pts], [p[1] for p in pts]
        self.root.update_idletasks()
        c = self.canvas
        w = c.winfo_width() if c.winfo_width() > 1 else c.winfo_reqwidth()
        h = c.winfo_height() if c.winfo_height() > 1 else c.winfo_reqheight()
        span_x, span_y = max(xs) - min(xs) + 6, max(ys) - min(ys) + 6
        self.scale = min(w / span_x, h / span_y)
        cx, cy = (max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2
        self.origin = (w / 2 - cx * self.scale, h / 2 + cy * self.scale)
        self.draw()

    def arrow(self, x, y, yaw, color, length_m=1.0, width=3):
        x0, y0 = self.to_px(x, y)
        x1, y1 = self.to_px(x + length_m * math.cos(yaw), y + length_m * math.sin(yaw))
        self.canvas.create_line(x0, y0, x1, y1, fill=color, width=width, arrow=tk.LAST)

    def draw(self):
        c = self.canvas
        c.delete("all")
        w, h = c.winfo_width(), c.winfo_height()
        x0, y1 = self.to_m(0, 0)
        x1, y0 = self.to_m(w, h)
        step = 1 if self.scale > 15 else 5
        for gx in range(math.floor(x0 / step) * step, math.ceil(x1) + 1, step):
            px = self.to_px(gx, 0)[0]
            c.create_line(px, 0, px, h, fill="#555" if gx == 0 else "#e8e8e8")
        for gy in range(math.floor(y0 / step) * step, math.ceil(y1) + 1, step):
            py = self.to_px(0, gy)[1]
            c.create_line(0, py, w, py, fill="#555" if gy == 0 else "#e8e8e8")
        c.create_text(
            8,
            h - 8,
            anchor="sw",
            fill="#555",
            text=f"grid {step} m, x right, y up (map frame)",
        )

        for t in self.ros.tracks:  # the live map
            lm = t.landmark
            cls = self.class_of.get((lm.type.value, lm.subtype.value), "?")
            col = self.color.get(cls, "#999")
            px, py = self.to_px(lm.pose.pose.position.x, lm.pose.pose.position.y)
            c.create_line(px - 5, py - 5, px + 5, py + 5, fill=col, width=2)
            c.create_line(px - 5, py + 5, px + 5, py - 5, fill=col, width=2)
            c.create_text(
                px + 6,
                py + 6,
                anchor="nw",
                fill=col,
                font=("TkDefaultFont", 7),
                text=f"{cls} n={t.observations}",
            )
        for e in self.prior.entries:
            col = self.color.get(e["class"], "black")
            px, py = self.to_px(e["x"], e["y"])
            r = e.get("sigma_xy", 1.0) * self.scale
            c.create_oval(px - r, py - r, px + r, py + r, outline=col, dash=(3, 3))
            c.create_oval(px - 5, py - 5, px + 5, py + 5, fill=col, outline="")
            c.create_text(px + 7, py - 7, anchor="sw", text=e["class"], fill=col)
        s = self.prior.start
        self.arrow(s["x"], s["y"], s["yaw"], "#2ca02c", 1.5, 4)
        sx, sy = self.to_px(s["x"], s["y"])
        c.create_text(sx, sy + 12, anchor="n", text="start", fill="#2ca02c")
        v = self.ros.vehicle()
        if v:
            self.arrow(*v, "black", 0.8, 3)

    def tick(self):
        if not self.drag:
            self.draw()
        self.root.after(500, self.tick)


def main():
    rclpy.init()
    ros = Ros()
    executor = SingleThreadedExecutor()
    executor.add_node(ros)
    spin = threading.Thread(target=executor.spin)
    spin.start()
    root = tk.Tk()
    Gui(root, ros)
    root.mainloop()
    executor.shutdown()
    spin.join()
    ros.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
