import pytest
import rclpy

from keyboard_joy.keyboard_joy_node import KeyboardJoy


class DummyListener:
    """Stand-in for pynput.keyboard.Listener that does nothing."""
    def __init__(self, on_press=None, on_release=None):
        self.on_press = on_press
        self.on_release = on_release
        self.started = False
        self.stopped = False

    def start(self):
        self.started = True

    def stop(self):
        self.stopped = True


class KeyChar:
    """Mimics pynput key with .char"""
    def __init__(self, char):
        self.char = char


class KeyName:
    """Mimics pynput special key with .name"""
    def __init__(self, name):
        self.name = name


@pytest.fixture(scope="function")
def rclpy_ctx():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture(scope="function")
def node(monkeypatch, rclpy_ctx):
    # Prevent real keyboard listener from starting
    import keyboard_joy.keyboard_joy_node as mod
    monkeypatch.setattr(mod.keyboard, "Listener", DummyListener)

    # Inject deterministic mappings (avoid filesystem / YAML / ament index)
    def fake_load_key_mappings(self):
        self.axis_mappings = {
            "w": [0, 1.0, "hold"],
            "s": [0, -1.0, "hold"],
            "e": [1, 1.0, "sticky"],
            "q": [1, -1.0, "sticky"],
        }
        self.button_mappings = {
            " ": 0,
            "Key.enter": 1,
        }
        self.axis_increment_rate = 0.01
        self.axis_increment_step = 0.2

    monkeypatch.setattr(KeyboardJoy, "load_key_mappings", fake_load_key_mappings)

    n = KeyboardJoy()
    yield n
    n.destroy_node()


def test_key_to_string_char_lowercases():
    assert KeyboardJoy.key_to_string(KeyChar("W")) == "w"
    assert KeyboardJoy.key_to_string(KeyChar("w")) == "w"


def test_key_to_string_special_key():
    assert KeyboardJoy.key_to_string(KeyName("enter")) == "Key.enter"


def test_button_press_and_release(node):
    node.on_press(KeyChar(" "))
    assert node.joy_msg.buttons[0] == 1

    node.on_release(KeyChar(" "))
    assert node.joy_msg.buttons[0] == 0

    node.on_press(KeyName("enter"))
    assert node.joy_msg.buttons[1] == 1

    node.on_release(KeyName("enter"))
    assert node.joy_msg.buttons[1] == 0


def test_hold_axis_moves_towards_target(node):
    node.on_press(KeyChar("w"))

    node.update_active_axes()
    assert node.joy_msg.axes[0] == pytest.approx(0.2, abs=1e-6)

    node.update_active_axes()
    assert node.joy_msg.axes[0] == pytest.approx(0.4, abs=1e-6)

    node.on_release(KeyChar("w"))
    assert node.joy_msg.axes[0] == pytest.approx(0.0, abs=1e-6)


def test_hold_axis_clamps_at_target(node):
    node.on_press(KeyChar("w"))

    for _ in range(10):
        node.update_active_axes()

    assert node.joy_msg.axes[0] == pytest.approx(1.0, abs=1e-6)


def test_sticky_axis_accumulates_and_clamps(node):
    for _ in range(10):
        node.on_press(KeyChar("e"))
        node.on_release(KeyChar("e"))

    assert node.joy_msg.axes[1] == pytest.approx(1.0, abs=1e-6)

    for _ in range(20):
        node.on_press(KeyChar("q"))
        node.on_release(KeyChar("q"))

    assert node.joy_msg.axes[1] == pytest.approx(-1.0, abs=1e-6)


def test_listener_is_stopped_on_destroy(node):
    assert isinstance(node.listener, DummyListener)
    assert node.listener.started is True

    node.destroy_node()
    assert node.listener.stopped is True
