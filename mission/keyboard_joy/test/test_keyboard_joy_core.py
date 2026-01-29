from keyboard_joy.keyboard_joy_core import AxisBinding, AxisMode, KeyboardJoyCore


def test_initializes_axes_and_buttons_to_max_index():
    core = KeyboardJoyCore(
        axis_mappings={
            "w": AxisBinding(axis=1, val=1.0, mode=AxisMode.HOLD),
            "a": AxisBinding(axis=5, val=-1.0, mode=AxisMode.HOLD),
        },
        button_mappings={
            "1": 0,
            "q": 7,
        },
    )

    state = core.get_state()
    assert len(state.axes) == 6
    assert len(state.buttons) == 8
    assert state.axes == [0.0] * 6
    assert state.buttons == [0] * 8


def test_hold_axis_ramps_toward_target_and_stops_at_target():
    core = KeyboardJoyCore(
        axis_mappings={"w": AxisBinding(axis=1, val=1.0, mode=AxisMode.HOLD)},
        button_mappings={},
        axis_increment_step=0.3,
    )

    core.press("w")
    core.update_active_axes()
    assert core.get_state().axes[1] == 0.3

    core.update_active_axes()
    assert core.get_state().axes[1] == 0.6

    core.update_active_axes()
    assert core.get_state().axes[1] == 0.9

    core.update_active_axes()
    assert core.get_state().axes[1] == 1.0

    core.update_active_axes()
    assert core.get_state().axes[1] == 1.0


def test_hold_axis_negative_ramps_down_and_stops_at_target():
    core = KeyboardJoyCore(
        axis_mappings={"s": AxisBinding(axis=0, val=-1.0, mode=AxisMode.HOLD)},
        button_mappings={},
        axis_increment_step=0.4,
    )

    core.press("s")
    core.update_active_axes()
    assert core.get_state().axes[0] == -0.4

    core.update_active_axes()
    assert core.get_state().axes[0] == -0.8

    core.update_active_axes()
    assert core.get_state().axes[0] == -1.0


def test_hold_axis_release_resets_axis_to_zero():
    core = KeyboardJoyCore(
        axis_mappings={"w": AxisBinding(axis=1, val=1.0, mode=AxisMode.HOLD)},
        button_mappings={},
        axis_increment_step=0.5,
    )

    core.press("w")
    core.update_active_axes()
    assert core.get_state().axes[1] == 0.5

    core.release("w")
    assert core.get_state().axes[1] == 0.0

    core.update_active_axes()
    assert core.get_state().axes[1] == 0.0


def test_sticky_axis_accumulates_in_steps_and_clamps():
    core = KeyboardJoyCore(
        axis_mappings={
            "i": AxisBinding(axis=2, val=1.0, mode=AxisMode.STICKY),
            "k": AxisBinding(axis=2, val=-1.0, mode=AxisMode.STICKY),
        },
        button_mappings={},
        axis_increment_step=0.25,
    )

    core.press("i")
    assert core.get_state().axes[2] == 0.25
    core.press("i")
    assert core.get_state().axes[2] == 0.5

    core.press("k")
    assert core.get_state().axes[2] == 0.25

    for _ in range(10):
        core.press("i")
    assert core.get_state().axes[2] == 1.0

    for _ in range(20):
        core.press("k")
    assert core.get_state().axes[2] == -1.0


def test_sticky_axis_release_does_not_reset():
    core = KeyboardJoyCore(
        axis_mappings={"i": AxisBinding(axis=0, val=1.0, mode=AxisMode.STICKY)},
        button_mappings={},
        axis_increment_step=0.5,
    )

    core.press("i")
    assert core.get_state().axes[0] == 0.5

    core.release("i")
    assert core.get_state().axes[0] == 0.5


def test_button_press_and_release_sets_button_state():
    core = KeyboardJoyCore(
        axis_mappings={},
        button_mappings={" ": 0, "q": 4},
    )

    core.press(" ")
    assert core.get_state().buttons[0] == 1
    core.release(" ")
    assert core.get_state().buttons[0] == 0

    core.press("q")
    assert core.get_state().buttons[4] == 1
    core.release("q")
    assert core.get_state().buttons[4] == 0


def test_button_press_extends_buttons_array_if_needed():
    core = KeyboardJoyCore(axis_mappings={}, button_mappings={"x": 12})
    assert len(core.get_state().buttons) == 13

    core2 = KeyboardJoyCore(axis_mappings={}, button_mappings={})
    assert len(core2.get_state().buttons) == 0
    core2.button_mappings["x"] = 12
    core2.press("x")
    assert len(core2.get_state().buttons) == 13
    assert core2.get_state().buttons[12] == 1


def test_unknown_keys_do_not_change_state():
    core = KeyboardJoyCore(
        axis_mappings={"w": AxisBinding(axis=0, val=1.0, mode=AxisMode.HOLD)},
        button_mappings={" ": 0},
    )
    before = core.get_state()

    core.press("does-not-exist")
    core.release("does-not-exist")
    core.update_active_axes()

    after = core.get_state()
    assert after.axes == before.axes
    assert after.buttons == before.buttons


def test_set_axis_clamps_and_rounds():
    core = KeyboardJoyCore(axis_mappings={}, button_mappings={})
    core.set_axis(3, 1.23456)
    assert core.get_state().axes[3] == 1.0

    core.set_axis(3, -1.23456)
    assert core.get_state().axes[3] == -1.0

    core.set_axis(3, 0.33333)
    assert core.get_state().axes[3] == 0.333
