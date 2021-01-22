import rospy

#from vortex_msgs.msg import ThrusterForces
from geometry_msgs.msg import Wrench
from sensor_msgs.msg import Joy


class JoystickMappingNode(object):

	def __init__(self):
		
		rospy.init_node('ControllerMapping', anonymous = True)

		self.sub = 
		rospy.Subscriber('manta/thruster_manager/input', Wrench, self.callback, queue_size=1)
		
		self.pub = 
		rospy.Publisher('joystick_control', Joy, queue_size=1)
		

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
	
		joystick_msg = Wrench()

		surge = joystick_msg.force.x
		sway = joystick_msg.force.y
		heave = joystick_msg.force.z	
		roll = joystick_msg.torque.x
		pitch = joystick_msg.torque.y
		yaw = joystick_msg.torque.z

		surge = axes['vertical_axis_left_stick']     # Surge
	   	sway = -axes['horizontal_axis_left_stick']
	   	heave = (axes['RT'] - axes['LT'])/2  
	    roll = (buttons['RB'] - buttons['LB'])      # Roll
	    pitch = -axes['vertical_axis_right_stick']   # Pitch
	    yaw = -axes['horizontal_axis_right_stick']  # Yaw
       
    	
    	self.pub.publish(joystick_msg)


if __name__ == '__main__':

	try:
		ControllerMapping = JoystickMappingNode()
		rospy.spin()

	except rospy.ROSInterruptException:
		pass

