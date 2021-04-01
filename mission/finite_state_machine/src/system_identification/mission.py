#!/usr/bin/env python

from system_identification.si_states import SingleTest
from common_states import create_sequence


def surge_sway_heave():
    states = [
        SingleTest(
            twist(1, 0, 0, 0, 0, 0), pose(0, 0, 0, 0, 0, 0), pose(5, 0, 0, 0, 0, 0)
        ),
        SingleTest(
            twist(1, 0, 0, 0, 0, 0), pose(0, 0, 0, 0, 0, 0), pose(5, 0, 0, 0, 0, 0)
        ),
        SingleTest(
            twist(1, 0, 0, 0, 0, 0), pose(0, 0, 0, 0, 0, 0), pose(5, 0, 0, 0, 0, 0)
        ),
        SingleTest(
            twist(1, 0, 0, 0, 0, 0), pose(0, 0, 0, 0, 0, 0), pose(5, 0, 0, 0, 0, 0)
        ),
        SingleTest(
            twist(1, 0, 0, 0, 0, 0), pose(0, 0, 0, 0, 0, 0), pose(5, 0, 0, 0, 0, 0)
        ),
    ]
    sm = create_sequence(states)
    sm.execute()


if __name__ == "__main__":
    surge_sway_heave()
