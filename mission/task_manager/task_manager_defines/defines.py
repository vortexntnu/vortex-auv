#!/usr/bin/python3

from dataclasses import dataclass


@dataclass
class Task:
    id: int
    name: str


class Tasks:
    joystick = Task(id=0, name="joystick")
    valve_vertical = Task(id=1, name="valve_vertical")
    valve_horisontal = Task(id=2, name="valve_horisontal")
    pipeline_inspection = Task(id=3, name="pipeline_inspection")

    tasks = [joystick, valve_vertical, valve_horisontal, pipeline_inspection]
