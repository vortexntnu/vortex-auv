from scripts.joystick_interface_auv import JoystickInterface 
from scripts.joystick_interface_auv import States
import rclpy
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Joy


class TestJoystickInterface:
    #test that the wrench msg is created successfully
    def test_wrench_msg(self):
        rclpy.init()
        msg = JoystickInterface().create_wrench_message(
            2.0, 3.0, 4.0, 5.0, 6.0, 7.0)
        assert msg.force.x == 2.0
        assert msg.force.y == 3.0
        assert msg.force.z == 4.0
        assert msg.torque.x == 5.0
        assert msg.torque.y == 6.0
        assert msg.torque.z == 7.0
        rclpy.shutdown()

    #Test that the callback function will be able to interpret the joy msg
    def test_input_from_controller_into_wrench_msg(self):
        rclpy.init()
        joy_msg = Joy()
        joy_msg.axes = [-1.0, -1.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
        joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        wrench_msg = JoystickInterface().joystick_cb(joy_msg)
        assert wrench_msg.force.x == -60.0
        assert wrench_msg.force.y == -60.0
        assert wrench_msg.torque.z == 0.0
        rclpy.shutdown()

    #When the killswitch button is activated in the buttons list, it should output a wrench msg with only zeros
    def test_killswitch_button(self):
        rclpy.init()
        joystick = JoystickInterface()
        joystick.state_ = States.XBOX_MODE
        joy_msg = Joy()
        joy_msg.axes = [-1.0, -1.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
        joy_msg.buttons = [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        wrench_msg = joystick.joystick_cb(joy_msg)
        assert wrench_msg.force.x == 0.0
        assert wrench_msg.force.y == 0.0
        assert wrench_msg.torque.z == 0.0
        rclpy.shutdown()

    #When we move into XBOX mode it should still be able to return this wrench message
    def test_moving_in_of_xbox_mode(self):
        rclpy.init()
        joystick = JoystickInterface()
        joystick.state_ = States.XBOX_MODE
        joy_msg = Joy()
        joy_msg.axes = [-1.0, -1.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
        joy_msg.buttons = [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        wrench_msg = joystick.joystick_cb(joy_msg)
        assert wrench_msg.force.x == -60.0
        assert wrench_msg.force.y == -60.0
        assert wrench_msg.torque.z == 0.0
        rclpy.shutdown()
