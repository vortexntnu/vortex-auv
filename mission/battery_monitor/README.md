## Battery Monitor

Running out of power in the middle of a mission is less than ideal, on top of possibly damaging the betteries and other components.

### Inputs

The node takes no arguments. The settings are set in the source-file as they are not supposed to be changed often.

### Outputs

This node publishes the battery level using the auv/battery_level publisher. It also logs the info through `rospy.loginfo`. If battery voltages drop beneath a threshold it will start using `rospy.logwarn` instead of `loginfo`. If it falls even lower it will write the voltage using `rospy.logfatal` 10 times, with half a second in between. Then it will shut down the drone.