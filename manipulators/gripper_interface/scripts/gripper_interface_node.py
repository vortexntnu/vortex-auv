#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
import Jetson.GPIO as GPIO

# TODO: Need different nodes/topics/modes for ROV and AUV operations
# TODO: Integrate with HW using https://github.com/NVIDIA/jetson-gpio

class GripperInterfaceNode():
    def __init__(self):
        rospy.init_node('/manipulators/gripper_interface')
        self.joystick_sub = rospy.Subscriber('/mission/joystick_data', Joy, self.callback, queue_size=1)
        self.gripper_state_pub = rospy.Publisher('/manipulators/gripper', Bool, queue_size=1)
        
        self.btn_state = 0
        self.prev_btn_state = 0

        self.gripper_state = 0

        # GPIO setup
        GPIO.setmode(GPIO.BOARD)

    def btn_bounceback(self):
        if self.btn_state and self.prev_btn_state:
            return True     
        elif self.btn_state and not self.prev_btn_state:
            return True
        elif not self.btn_state and self.prev_btn_state:
            return True
        

    def callback(data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        


if __name__ == '__main__':
    node = GripperInterfaceNode()
    
    while not rospy.is_shutdown():
        pass