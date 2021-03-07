# Some code to test if we get reasonable results from the joystick-input
#!/usr/bin/env python
#from __future__ import division

from geometry_msgs.msg import Wrench
from sensor_msgs.msg import Joy
from math import sqrt
#from __future__ import division



min_point_range = 0.2
max_point_range = 1.0



min_point_range = 0.2                          # Minimum allowed distance from the UUV to the point
max_point_range = 1.0                          # Maximum allowed distance from the UUV to the calculated point
num_ranges = 5                                 # Number of ranges the input is scaled by
joystick_num_bit = 16
num_values = 32767.0

deadzone = 1000.0 # Only used for the upper value


# The limits are used to calculate the actual limits 
limits = []
max_value = pow(2, joystick_num_bit - 1)  # -1 due to only positive integers
for i in range(1, num_ranges + 1):
    print((max_value / float(num_ranges)) * i)          
    limits.append((max_value / float(num_ranges)) * i)

# The actual values that we set the values to
decimal_values = []
for i in range(0, num_ranges + 1):
    decimal_values.append((max_point_range / float(num_ranges)) * i)


def nearest_list_value(value, ref_list):
	"""
	Returns the highest value in the list 'ref_list' that is
	less or equal to the given value 'value'.
	"""

	# Sorting the list in ascending order
	ref_list = sorted(ref_list, key = lambda x:float(x)) 

	# Using a temp_value to keep track of sign
	temp_value = 0

	for i in range(len(ref_list)):
		if abs(value) < ref_list[i]:
			temp_value = decimal_values[i]
			break

    # We allow maximum thrust when it is given below a certain threshold
	if abs(value) >= ref_list[-1] - deadzone:
		# We know that we are given largest value in the list
		temp_value = decimal_values[-1]

	# Correcting for negative sign
	if value < 0:
		temp_value *= -1 

	value = temp_value
	return value

def calculate_joystick_point(joystick_msg):
    """
    Calculates a point in the local frame based on the given thrust-vectors 
    from joystick 

    If required it must be converted into the global reference-frame
    """

    # Scaling the force to get a linearized model
    scaled_force_x, scaled_force_y, scaled_force_z = scale_force_vectors(joystick_msg)
    calculated_point = [scaled_force_x, scaled_force_y, scaled_force_z]

    # Calculating the length of the vectors
    vector_length_square = 0
    for i in range(len(calculated_point)):
        vector_length_square += pow(calculated_point[i], 2)

    # Normalizing if exceeding the max_point_range
    if vector_length_square >= pow(max_point_range, 2):
        # Over the set limit. Normalizing
        print("Length of the vector " + str(vector_length_square))
        vector_length = sqrt(vector_length_square)
        calculated_point = [val / vector_length for val in calculated_point]

    return calculated_point


def scale_force_vectors(joystick_msg):
    """
    Scales the thrust-vectors to
    """

    # Recovering the given forces
    scale_x_vec = joystick_msg.force.x
    scale_y_vec = joystick_msg.force.y
    scale_z_vec = joystick_msg.force.z

    # Using a try-catch to prevent out-of-bounds to become a large problem
    try:
        # Scale each force
        scale_x_vec = nearest_list_value(scale_x_vec, limits)
        scale_y_vec = nearest_list_value(scale_y_vec, limits)
        scale_z_vec = nearest_list_value(scale_z_vec, limits)

        return scale_x_vec, scale_y_vec, scale_z_vec

    except Exception as e:
        print(e)
        # rospy.logerr(e)

        # Return a standard response if an error occurs
        return 0, 0, 0



values_x = [0, 25671, 3909, 1533]
values_y = [583, 8262, 20000, 100]
values_z = [6500, 9873, 23543, 0]

vector_lengths = []
results = []

for i in range(len(values_x)):
    
    joystick_msg = Wrench()
    joystick_msg.force.x = values_x[i]
    joystick_msg.force.y = values_y[i]
    joystick_msg.force.z = values_z[i]   

    point = calculate_joystick_point(joystick_msg)
    print(point)
    


