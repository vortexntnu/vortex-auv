import rospy
from geometry_msgs.msg import Wrench
from sensor_msgs.msg import Joy


class JoystickMappingNode(object):

	def __init__(self):

		rospy.init_node('joystick_mapping', anonymous = True)

		self.pub = 
		rospy.Publisher('manta/thruster_manager/input', Wrench, queue_size=1)
		
		self.sub = 
		rospy.Subscriber('joystick_control', Joy, self.callback, queue_size=1)
		

		# Name buttons and axes based on index from joy-node
        self.buttons_map = ['A', 'B', 'X', 'Y', 'LB', 'RB', 'back',
                            'start', 'power', 'stick_button_left',
                            'stick_button_right']

        self.axes_map = ['horizontal_axis_left_stick',
                         'vertical_axis_left_stick', 'LT',
                         'horizontal_axis_right_stick',
                         'vertical_axis_right_stick', 'RT',
                         'dpad_horizontal', 'dpad_vertical']


		def callback(self, msg):
	
		buttons = {}
		axes = {}

		for i in range(len(msg.buttons)):
			buttons[self.buttons_map[i]] = msg.buttons[i]

		for j in range(len(msg.axes)):
            axes[self.axes_map[j]] = msg.axes[j]

		joystick_msg = Wrench()

		surge = axes['vertical_axis_left_stick']     
	   	sway = -axes['horizontal_axis_left_stick']	
	   	heave = (axes['RT'] - axes['LT'])/2  		 
	    roll = (buttons['RB'] - buttons['LB'])       
	    pitch = -axes['vertical_axis_right_stick']  
	    yaw = -axes['horizontal_axis_right_stick']  
    	
	   	joystick_msg.force.x = surge
		joystick_msg.force.y = sway
		joystick_msg.force.z = heave
		joystick_msg.torque.x = roll
		joystick_msg.torque.y = pitch
		joystick_msg.torque.z = yaw

    	self.pub.publish(joystick_msg)


if __name__ == '__main__':

	try:
		joystick_mapping = JoystickMappingNode()
		rospy.spin()

	except rospy.ROSInterruptException:
		pass

