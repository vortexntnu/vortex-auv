# This message contains a Vortex ROV propulsion command

Header header

# Normalized motion command
#
# All elements must be in the range [-1, 1]
# The order is
# [forward, right, down, roll right, tilt up, turn right]
float64[6] motion

# Control mode
#
# Set all elements false to keep current mode.
# Set one element true to switch mode. Its index determines the mode,
# as defined in the control_modes.h.
bool[] control_mode
