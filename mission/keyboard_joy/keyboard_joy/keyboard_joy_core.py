# keyboard_joy_core.py
from __future__ import annotations

import threading
from dataclasses import dataclass
from enum import Enum

import yaml


class AxisMode(Enum):
    HOLD = "hold"
    STICKY = "sticky"


@dataclass(frozen=True)
class AxisBinding:
    axis: int
    val: float
    mode: AxisMode


@dataclass(frozen=True)
class JoyState:
    axes: list[float]
    buttons: list[int]
    frame_id: str = "keyboard"


class KeyboardJoyCore:
    def __init__(
        self,
        axis_mappings: dict[str, AxisBinding],
        button_mappings: dict[str, int],
        *,
        axis_update_period: float = 0.02,
        publish_period: float = 0.05,
        axis_increment_step: float = 0.05,
        frame_id: str = "keyboard",
    ):
        self.axis_mappings = axis_mappings
        self.button_mappings = button_mappings

        self.axis_update_period = float(axis_update_period)
        self.publish_period = float(publish_period)
        self.axis_increment_step = float(axis_increment_step)

        self._frame_id = frame_id

        self._active_axes: dict[int, float] = {}
        self._sticky_axes: dict[int, float] = {}

        max_axis_index = max((b.axis for b in self.axis_mappings.values()), default=-1)
        max_button_index = max(self.button_mappings.values(), default=-1)

        self._axes = [0.0] * (max_axis_index + 1)
        self._buttons = [0] * (max_button_index + 1)

        self._lock = threading.Lock()

    @classmethod
    def from_yaml_file(cls, config_file: str) -> KeyboardJoyCore:
        with open(config_file, encoding="utf-8") as f:
            keymap = yaml.safe_load(f) or {}

        raw_axes = keymap.get("axes", {}) or {}
        axis_mappings: dict[str, AxisBinding] = {}
        for key, spec in raw_axes.items():
            # YAML format: [axis_index, value, "sticky"/"hold"]
            axis, val, mode = spec
            axis_mappings[key] = AxisBinding(
                axis=int(axis),
                val=float(val),
                mode=AxisMode(str(mode)),
            )

        button_mappings = keymap.get("buttons", {}) or {}

        params = keymap.get("parameters", {}) or {}
        axis_update_period = float(params.get("axis_update_period", 0.02))
        publish_period = float(params.get("publish_period", 0.05))
        axis_increment_step = float(params.get("axis_increment_step", 0.05))

        return cls(
            axis_mappings=axis_mappings,
            button_mappings=button_mappings,
            axis_update_period=axis_update_period,
            publish_period=publish_period,
            axis_increment_step=axis_increment_step,
        )

    def press(self, key_str: str) -> None:
        if not key_str:
            return

        with self._lock:
            if key_str in self.axis_mappings:
                binding = self.axis_mappings[key_str]

                if binding.mode == AxisMode.STICKY:
                    axis = binding.axis
                    new_val = (
                        self._sticky_axes.get(axis, 0.0)
                        + binding.val * self.axis_increment_step
                    )
                    new_val = max(min(new_val, 1.0), -1.0)
                    self._sticky_axes[axis] = new_val
                    self._axes[axis] = round(new_val, 3)
                else:
                    # HOLD mode: ramp toward binding.val while key is held
                    self._active_axes[binding.axis] = binding.val

            elif key_str in self.button_mappings:
                idx = int(self.button_mappings[key_str])
                if idx >= len(self._buttons):
                    self._buttons.extend([0] * (idx + 1 - len(self._buttons)))
                self._buttons[idx] = 1

    def release(self, key_str: str) -> None:
        if not key_str:
            return

        with self._lock:
            if key_str in self.axis_mappings:
                binding = self.axis_mappings[key_str]
                self._active_axes.pop(binding.axis, None)
                if binding.mode != AxisMode.STICKY:
                    self._axes[binding.axis] = 0.0

            elif key_str in self.button_mappings:
                idx = int(self.button_mappings[key_str])
                if idx < len(self._buttons):
                    self._buttons[idx] = 0

    def update_active_axes(self) -> None:
        """For HOLD axes: move axis value toward target at axis_increment_step per call."""
        with self._lock:
            for axis, target in list(self._active_axes.items()):
                if axis >= len(self._axes):
                    self._axes.extend([0.0] * (axis + 1 - len(self._axes)))

                current = self._axes[axis]
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

                self._axes[axis] = round(next_val, 3)

    def get_state(self) -> JoyState:
        with self._lock:
            return JoyState(
                axes=list(self._axes),
                buttons=list(self._buttons),
                frame_id=self._frame_id,
            )

    # Helper for tests/debug
    def set_axis(self, axis: int, value: float) -> None:
        with self._lock:
            if axis >= len(self._axes):
                self._axes.extend([0.0] * (axis + 1 - len(self._axes)))
            self._axes[axis] = round(max(min(float(value), 1.0), -1.0), 3)
