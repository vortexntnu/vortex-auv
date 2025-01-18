from joystick_utils import *

class JoystickInterface:
    def __init__(self, joystick_gains: State, guidance_gains: State):
        self.joystick_gains = joystick_gains
        self.guidance_gains = guidance_gains

    def calculate_movement(self, buttons: dict, axes: dict) -> State:
        state = State()

        left_trigger = axes.get("LT", 0.0)
        right_trigger = axes.get("RT", 0.0)
        right_shoulder = buttons.get("RB", 0)
        left_shoulder = buttons.get("LB", 0)

        state.x = axes.get(
            "vertical_axis_left_stick", 0.0
        ) * self.joystick_gains.x

        state.y = -axes.get(
            "horizontal_axis_left_stick", 0.0
        ) * self.joystick_gains.y

        state.z = -(
            left_trigger - right_trigger
        ) * self.joystick_gains.z

        state.roll = (
            right_shoulder - left_shoulder
        ) * self.joystick_gains.roll

        state.pitch = -axes.get(
            "vertical_axis_right_stick", 0.0
        ) * self.joystick_gains.pitch

        state.yaw = -axes.get(
            "horizontal_axis_right_stick", 0.0
        ) * self.joystick_gains.yaw

        return state
    
    def update_reference_state(self, state: State, movement: State) -> State:
        state.x += movement.x / self.guidance_gains.x
        state.y += movement.y / self.guidance_gains.y
        state.z += movement.z / self.guidance_gains.z
        state.roll += movement.roll / self.guidance_gains.roll
        state.pitch += movement.pitch / self.guidance_gains.pitch
        state.yaw += movement.yaw / self.guidance_gains.yaw

        return state