class JoyStates:
    XBOX_MODE = "xbox"
    AUTONOMOUS_MODE = "autonomous"
    KILLSWITCH = "killswitch"
    REFERENCE_MODE = "reference"


class Wired:
    joystick_buttons_map_ = [
        "A",
        "B",
        "X",
        "Y",
        "LB",
        "RB",
        "back",
        "start",
        "power",
        "stick_button_left",
        "stick_button_right",
        "share_button",
    ]

    joystick_axes_map_ = [
        "horizontal_axis_left_stick",
        "vertical_axis_left_stick",
        "LT",
        "horizontal_axis_right_stick",
        "vertical_axis_right_stick",
        "RT",
        "dpad_horizontal",
        "dpad_vertical",
    ]


class WirelessXboxSeriesX:
    joystick_buttons_map_ = [
        "A",
        "B",
        "0",
        "X",
        "Y",
        "0",
        "LB",
        "RB",
        "0",
        "0",
        "back",
        "start",
        "power",
        "stick_button_left",
        "stick_button_right",
        "share_button",
    ]

    joystick_axes_map_ = [
        "horizontal_axis_left_stick",
        "vertical_axis_left_stick",
        "horizontal_axis_right_stick",
        "vertical_axis_right_stick",
        "RT",
        "LT",
        "dpad_horizontal",
        "dpad_vertical",
    ]
