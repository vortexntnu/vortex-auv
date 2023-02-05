#!/usr/bin/python3

from dataclasses import dataclass


@dataclass
class Task:
    id: int
    name: str


class Tasks:
    test_1              = Task(id=0, name="test_1")
    test_2              = Task(id=1, name="test_2")
    joystick            = Task(id=2, name="joystick")
    valve_vertical      = Task(id=3, name="valve_vertical")
    valve_horisontal    = Task(id=4, name="valve_horisontal")
    pipeline_inspection = Task(id=5, name="pipeline_inspection")