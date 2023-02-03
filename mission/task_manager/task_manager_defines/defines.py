#!/usr/bin/python3

from dataclasses import dataclass

@dataclass
class Task:
    id: int
    name: str

class Tasks:
    test_1 = Task(id=0, name="test_1")
    test_2 = Task(id=1, name="test_2")
