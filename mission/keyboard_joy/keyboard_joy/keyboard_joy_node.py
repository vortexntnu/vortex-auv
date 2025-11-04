#!/usr/bin/env python3
import os
import threading

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from pynput import keyboard
from rclpy.node import Node
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


class KeyboardJoy(Node):
    def __init__(self):
        super().__init__('keyboard_joy')

        # Load configuration
        self.declare_parameter('config', '')
        self.load_key_mappings()

        # Prepare publisher
        self.joy_pub = self.create_publisher(Joy, 'joy', 10)

        # Precompute max indices for axes/buttons
        max_axis_index = max((v[0] for v in self.axis_mappings.values()), default=-1)
        max_button_index = max(self.button_mappings.values(), default=-1)

        self.joy_msg = Joy()
        self.joy_msg.axes = [0.0] * (max_axis_index + 1)
        self.joy_msg.buttons = [0] * (max_button_index + 1)

        self.active_axes = {}
        self.sticky_axes = {}

        self.lock = threading.Lock()

        # Start keyboard listener
        self.listener = keyboard.Listener(
            on_press=self.on_press, on_release=self.on_release
        )
        self.listener.start()

        self.create_timer(0.05, self.publish_joy)
        self.create_timer(self.axis_increment_rate, self.update_active_axes)

        self.get_logger().info(start_message)

    def load_key_mappings(self):
        config_file = self.get_parameter(
            'config'
        ).get_parameter_value().string_value or os.path.join(
            get_package_share_directory('keyboard_joy'), 'config', 'key_mappings.yaml'
        )

        try:
            with open(config_file) as f:
                keymap = yaml.safe_load(f) or {}
        except Exception as e:
            self.get_logger().error(f"Failed to load config '{config_file}': {e}")
            keymap = {}

        self.axis_mappings = keymap.get('axes', {})
        self.button_mappings = keymap.get('buttons', {})
        params = keymap.get('parameters', {})

        self.axis_increment_rate = float(params.get('axis_increment_rate', 0.02))
        self.axis_increment_step = float(params.get('axis_increment_step', 0.05))

    def on_press(self, key):
        key_str = self.key_to_string(key)
        if not key_str:
            return
        with self.lock:
            if key_str in self.axis_mappings:
                axis, val, mode = self.axis_mappings[key_str]
                if mode == 'sticky':
                    self.sticky_axes[axis] = max(
                        min(
                            self.sticky_axes.get(axis, 0.0)
                            + val * self.axis_increment_step,
                            1.0,
                        ),
                        -1.0,
                    )
                    self.joy_msg.axes[axis] = round(self.sticky_axes[axis], 3)
                else:
                    self.active_axes[axis] = val
            elif key_str in self.button_mappings:
                self.joy_msg.buttons[self.button_mappings[key_str]] = 1

    def on_release(self, key):
        key_str = self.key_to_string(key)
        if not key_str:
            return
        with self.lock:
            if key_str in self.axis_mappings:
                axis, _, mode = self.axis_mappings[key_str]
                self.active_axes.pop(axis, None)
                if mode != 'sticky':
                    self.joy_msg.axes[axis] = 0.0
            elif key_str in self.button_mappings:
                self.joy_msg.buttons[self.button_mappings[key_str]] = 0

    @staticmethod
    def key_to_string(key):
        """Convert pynput key event to a normalized string."""
        if hasattr(key, 'char') and key.char:
            return key.char.lower()
        if hasattr(key, 'name') and key.name:
            return f'Key.{key.name}'
        return None

    def update_active_axes(self):
        """Gradually update active axes towards their target values."""
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


if __name__ == '__main__':
    main()
