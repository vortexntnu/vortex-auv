import rospy

from geometry_msgs.msg import Wrench
from sensor_msgs.msg import Joy

# to configure joystick environment, please refer to http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick

class JoystickMappingNode(object):

	def __init__(self):
		
		rospy.init_node('joystick_control', anonymous = True)

		self.sub = 
		rospy.Subscriber('joystick_node', Wrench, self.callback, queue_size=1)
		
		self.pub = 
		rospy.Publisher('manta/thruster_manager/input', Wrench, queue_size=1)
		

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

		surge = joystick_msg.force.x
		sway = joystick_msg.force.y
		heave = joystick_msg.force.z	
		roll = joystick_msg.torque.x
		pitch = joystick_msg.torque.y
		yaw = joystick_msg.torque.z

		axes['vertical_axis_left_stick'] = surge    
	   	-axes['horizontal_axis_left_stick'] = sway
	   	(axes['RT'] - axes['LT'])/2 = heave
	    (buttons['RB'] - buttons['LB']) = roll     
	    -axes['vertical_axis_right_stick'] = pitch   
	    -axes['horizontal_axis_right_stick'] = yaw  
       
    	
    	self.pub.publish(joystick_msg)


if __name__ == '__main__':

	try:
		joystick_control = JoystickMappingNode()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

