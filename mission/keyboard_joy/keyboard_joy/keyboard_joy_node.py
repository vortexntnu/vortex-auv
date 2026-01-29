#!/usr/bin/env python3
import rclpy
from pynput import keyboard
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Joy

from keyboard_joy.keyboard_joy_core import KeyboardJoyCore

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
        super().__init__("keyboard_joy")

        self.declare_parameter("config", Parameter.Type.STRING)
        config_file = self.get_parameter("config").value
        if not config_file:
            raise RuntimeError("Parameter 'config' is required (pass it from launch).")

        self.core = KeyboardJoyCore.from_yaml_file(config_file)

        self.declare_parameter("topics.joy", Parameter.Type.STRING)
        joy_topic = self.get_parameter("topics.joy").value
        self.joy_pub = self.create_publisher(Joy, joy_topic, 10)

        self.joy_msg = Joy()
        self.joy_msg.header.frame_id = "keyboard"

        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release,
        )
        self.listener.start()

        self.create_timer(self.core.publish_period, self.publish_joy)
        self.create_timer(self.core.axis_update_period, self.update_active_axes)

        self.get_logger().info(start_message)

    def on_press(self, key):
        key_str = self.key_to_string(key)
        if not key_str:
            return
        self.core.press(key_str)

    def on_release(self, key):
        key_str = self.key_to_string(key)
        if not key_str:
            return
        self.core.release(key_str)

    @staticmethod
    def key_to_string(key):
        if hasattr(key, "char") and key.char:
            return key.char.lower()
        if hasattr(key, "name") and key.name:
            return f"Key.{key.name}"
        return None

    def update_active_axes(self):
        self.core.update_active_axes()

    def publish_joy(self):
        state = self.core.get_state()

        self.joy_msg.header.stamp = self.get_clock().now().to_msg()
        self.joy_msg.header.frame_id = state.frame_id
        self.joy_msg.axes = state.axes
        self.joy_msg.buttons = state.buttons
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
