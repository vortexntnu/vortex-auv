# Internal Status

This package contains nodes for retrieving internal statuses like battery levels and temperature. It also contains a monitoring script connected to an OLED screen which runs on boot on the Xavier, some error handling is explained at the bottom of this readme.

#### Temperature
High temperatures can be dangerous for any electronic device. Monitoring the temperature is therefore important. Temperature gets published to auv/temperature. Measured in celsius

#### Battery
Running out of power in the middle of a mission is less than ideal, on top of possibly damaging the betteries and other components.

This node publishes the battery level using the auv/battery_level publisher. It also logs the info through `rospy.loginfo`. If battery voltages drop beneath a threshold it will start using `rospy.logwarn` instead of `rospyloginfo`. If it falls even lower it will write the voltage using `rospy.logerror`. Measured in volt

## Inputs and Outputs

#### Publishes to:
* /auv/battery_level/xavier of type Int32 in volt
* /auv/battery_level/system of type Int32 in volt
* /auv/temperature of type Int32 in celsius

## Setup instructions

#### Parameters
In beluga.yaml

* `battery/threshold/warning` dictates at what voltage-level the node changes to `rospy.logwarn`

* `battery/threshold/critical` dictates at what voltage-level the node changes to `rospy.logfatal`

* `battery/logging/path` is the path to the Xavier-specific folder from where the voltage-levels are retrieved see more at [docs.nvidia.com](https://docs.nvidia.com/jetson/l4t/index.html#page/Tegra%20Linux%20Driver%20Package%20Development%20Guide/power_management_jetson_xavier.html#wwpID0E0AG0HA)

* `battery/logging/powersense_dev` is the path to powersense

* `temperature/logging/zones/` are the names of the zones which we want to monitor

* `*/logging/interval` dictates how often the node updates the voltage/temperature, the time given is how many seconds between each update

## OLED errors
Xavier IP, Xavier voltage, system voltage and number of users should be displayed on an OLED screen while the Xavier is powered on.
This is a script which runs on boot, seperate from ROS. 
If the screen doesnt show anything, try manually running bootscripts/display_battery_IP.py.
If you get an "remote IO" error, run the following terminal command:
```
i2cdetect -y -r 1
```
The expected output is:
```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- UU -- -- -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- 3c -- -- -- 
40: UU UU -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
70: -- -- -- -- 74 -- -- --                         
```
Any deviation from this output suggests that the pins are not connected right.
For I2C-bus 1 the SDA and CLK pins should be connected to pin 27 and 28 respectively.
Xavier pin layout (pin 1 is top right when looking at the Xavier):
![Xavier pin layout](https://aws1.discourse-cdn.com/nvidia/original/2X/b/bf92a41569803336a14c2f18cab74ba2496d0cfa.png)

############################################################################################################################
Adafruit library used to handle all OLED setup
IP Address of Raspberry pi is correctly displayed, and updating
To be tested:
IP address of Xavier handled through ROS publisher in Xavier, subscriber in Raspberry pi, which overwrites IP file in Raspberry pi, display script accesses the file and prints.
Voltage and current read function implemented, needs to be tested