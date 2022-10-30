#!/usr/bin/python3
# Voltage: 3.234 # Real 15.78
# Current: 0.7809 # Real

import time
import traceback

import smbus

if __name__ == "__main__":
    # Calibration values for converting from raw digital binary form to decimal form
    # Calibration values were manualy calibrated to +- 0.1V acuracy!
    calVoltageA = 0.0601
    calVoltageB = 0.3401
    calCurrent = 0.011

    # init of I2C bus with arduino nano conected
    bus = smbus.SMBus(1)
    time.sleep(1)

    nano_addr = 12  # I2C adress of nano (setted in software!)
    voltage_reg_nano = 0  # value to send to arduino to get voltage read back
    current_reg_nano = 1  # to get current measurement back

    while True:
        try:
            voltage_msg = bus.read_i2c_block_data(nano_addr, 0, 2)

            # conversion to get real voltage
            # measurement up to 1023, so to big for 7bit I2C messages. Sends MSB first, then LSB
            x = float((((voltage_msg[0] & 0x7) << 7) + voltage_msg[1]))
            system_voltage = x * calVoltageA + calVoltageB

            # debug
            print("System voltage raw : " + str(voltage_msg))
            print("System voltage reconverted : " + str(system_voltage))
        except:
            print("Bus error")

        try:
            current_msg = bus.read_i2c_block_data(nano_addr, 1, 2)

            # conversion to get real voltage
            system_current = (
                float((((current_msg[0] & 0x7) << 7) + current_msg[1])) * calCurrent
            )

            # debug
            print("System current raw : " + str(current_msg))
            print("System current reconverted: " + str(system_current))
        except:
            print("BUS error")
        time.sleep(1)  # sleep for 500 ms
