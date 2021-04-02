# Message for controlling lights on ROV
# String contains the name of the light to control,
# the intensity contains the desired intensity of the light.
# For pwm-controlled light,intensity is in the range 0-100, while for bluetooth/raman intensity 100 
# means on, 0 means off.
string toggle_light
uint8 intensity
