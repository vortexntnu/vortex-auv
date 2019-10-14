# Sensor Interface
These nodes interface with the IMU and the pressure sensors.

## Dependencies
* [Adafruit python library](https://github.com/adafruit/Adafruit_Python_BNO055) for BNO055.
* Python SMBus library for MS5837:
`sudo apt install python-smbus`


## Physical connections
The BNO055 library uses by default i2c-1, which has SCL on P9_17 and SDA on P9_18. RST is connected to P9_12.

