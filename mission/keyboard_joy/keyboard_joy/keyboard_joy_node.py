#!/usr/bin/env python3
import threading
from dataclasses import dataclass
from enum import Enum

import rclpy
import yaml
from pynput import keyboard
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Joy

start_message = r"""
 ██ ▄█▀▓█████▓██   ██▓ ▄▄▄▄    ▒█████   ▄▄▄       ██▀███  ▓█████▄  ▄▄▄██▀▀▀▒█████ ▓██   ██▓
 ██▄█▒ ▓█   ▀ ▒██  ██▒▓█████▄ ▒██▒  ██▒▒████▄    ▓██ ▒ ██▒▒██▀ ██▌   ▒██  ▒██▒  ██▒▒██  ██▒
▓███▄░ ▒███    ▒██ ██░▒██▒ ▄██▒██░  ██▒▒██  ▀█▄  ▓██ ░▄█ ▒░██   █▌   ░██  ▒██░  ██▒ ▒██ ██░
▓██ █▄ ▒▓█  ▄  ░ ▐██▓░▒██░█▀  ▒██   ██░░██▄▄▄▄██ ▒██▀▀█▄  ░▓█▄   ▌▓██▄██▓ ▒██   ██░ ░ ▐██▓░
▒██▒ █▄░▒████▒ ░ ██▒▓░░▓█  ▀█▓░ ████▓▒░ ▓█   ▓██▒░██▓ ▒██▒░▒████▓  ▓███▒  ░ ████▓▒░ ░ ██▒▓░
▒ ▒▒ ▓▒░░ ▒░ ░  ██▒▒▒ ░▒▓███▀▒░ ▒░▒░▒░  ▒▒   ▓▒█░░ ▒▓ ░▒▓░ ▒▒▓  ▒  ▒▓▒▒░  ░ ▒░▒░▒░   ██▒▒▒
░ ░▒ ▒░ ░ ░  ░▓██ ░▒░ ▒░▒   ░   ░ ▒ ▒░   ▒   ▒▒ ░  ░▒ ░ ▒░ ░ ▒  ▒  ▒ ░▒░    ░ ▒ ▒░ ▓██ ░▒░
░ ░░ ░    ░   ▒ ▒ ░░   ░    ░ ░ ░ ░ ▒    ░   ▒     ░░   ░  ░ ░  ░  ░ ░ ░  ░ ░ ░ ▒  ▒ ▒ ░░
░  ░      ░  ░░ ░      ░          ░ ░        ░  ░   ░        ░     ░   ░      ░ ░  ░ ░
              ░ ░           ░                              ░                       ░ ░
"""


class AxisMode(Enum):
    HOLD = "hold"
    STICKY = "sticky"


@dataclass(frozen=True)
class AxisBinding:
    axis: int
    val: float
    mode: AxisMode


class KeyboardJoy(Node):
    def __init__(self):
        super().__init__("keyboard_joy")

        self.declare_parameter("config", Parameter.Type.STRING)
        self.load_key_mappings()

        self.declare_parameter("topics.joy", Parameter.Type.STRING)
        joy_topic = self.get_parameter("topics.joy").value
        self.joy_pub = self.create_publisher(Joy, joy_topic, 10)

        self._init_joy_message()

        self.active_axes = {}
        self.sticky_axes = {}

        self.lock = threading.Lock()

        self.listener = keyboard.Listener(
            on_press=self.on_press, on_release=self.on_release
        )
        self.listener.start()

        self.create_timer(self.publish_period, self.publish_joy)
        self.create_timer(self.axis_update_period, self.update_active_axes)

        self.get_logger().info(start_message)

    def load_key_mappings(self):
        config_file = self.get_parameter("config").value
        if not config_file:
            raise RuntimeError("Parameter 'config' is required (pass it from launch).")

        try:
            with open(config_file) as f:
                keymap = yaml.safe_load(f) or {}
        except Exception as e:
            self.get_logger().error(f"Failed to load config '{config_file}': {e}")
            keymap = {}

        raw_axes = keymap.get("axes", {}) or {}
        self.axis_mappings = {}
        for key, spec in raw_axes.items():
            # YAML format: [axis_index, value, "sticky"/"hold"]
            axis, val, mode = spec
            self.axis_mappings[key] = AxisBinding(
                axis=int(axis),
                val=float(val),
                mode=AxisMode(mode),
            )

        self.button_mappings = keymap.get("buttons", {}) or {}

        params = keymap.get("parameters", {}) or {}
        self.axis_update_period = float(params.get("axis_update_period", 0.02))
        self.publish_period = float(params.get("publish_period", 0.05))
        self.axis_increment_step = float(params.get("axis_increment_step", 0.05))

    def _init_joy_message(self):
        # Find the highest axis index used in the key mappings (0-based)
        max_axis_index = max((b.axis for b in self.axis_mappings.values()), default=-1)

        # Find the highest button index used in the key mappings (0-based)
        max_button_index = max(self.button_mappings.values(), default=-1)

        self.joy_msg = Joy()
        self.joy_msg.header.frame_id = "keyboard"

        # Allocate arrays large enough to cover the highest index (+1 because 0-based)
        self.joy_msg.axes = [0.0] * (max_axis_index + 1)
        self.joy_msg.buttons = [0] * (max_button_index + 1)

    def on_press(self, key):
        key_str = self.key_to_string(key)
        if not key_str:
            return

        with self.lock:
            if key_str in self.axis_mappings:
                binding = self.axis_mappings[key_str]

                if binding.mode == AxisMode.STICKY:
                    axis = binding.axis
                    self.sticky_axes[axis] = max(
                        min(
                            self.sticky_axes.get(axis, 0.0)
                            + binding.val * self.axis_increment_step,
                            1.0,
                        ),
                        -1.0,
                    )
                    self.joy_msg.axes[axis] = round(self.sticky_axes[axis], 3)
                else:
                    self.active_axes[binding.axis] = binding.val

            elif key_str in self.button_mappings:
                self.joy_msg.buttons[self.button_mappings[key_str]] = 1

    def on_release(self, key):
        key_str = self.key_to_string(key)
        if not key_str:
            return

        with self.lock:
            if key_str in self.axis_mappings:
                binding = self.axis_mappings[key_str]
                self.active_axes.pop(binding.axis, None)
                if binding.mode != AxisMode.STICKY:
                    self.joy_msg.axes[binding.axis] = 0.0

            elif key_str in self.button_mappings:
                self.joy_msg.buttons[self.button_mappings[key_str]] = 0

    @staticmethod
    def key_to_string(key):
        if hasattr(key, "char") and key.char:
            return key.char.lower()
        if hasattr(key, "name") and key.name:
            return f"Key.{key.name}"
        return None

    def update_active_axes(self):
        with self.lock:
            for axis, target in self.active_axes.items():
                current = self.joy_msg.axes[axis]
                delta = (
                    self.axis_increment_step
                    if target > 0
                    else -self.axis_increment_step
                )
                next_val = current + delta
                if (delta > 0 and next_val > target) or (
                    delta < 0 and next_val < target
                ):
                    next_val = target
                self.joy_msg.axes[axis] = round(next_val, 3)

    def publish_joy(self):
        with self.lock:
            self.joy_msg.header.stamp = self.get_clock().now().to_msg()
            self.joy_pub.publish(self.joy_msg)

    def destroy_node(self):
        if self.listener:
            self.listener.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardJoy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
