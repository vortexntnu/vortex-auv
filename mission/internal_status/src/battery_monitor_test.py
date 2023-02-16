#!/usr/bin/python3

import os
import time
import traceback
import glob
import logging
import smbus
import board

from mcp3422 import MCP3422


class BatteryMonitor:
    def __init__(self):

        # Parameters
        # to read voltage and current from ADC on PDB through I2C
        self.i2c_adress = 0x69

        # init of I2C bus communication
        self.bus = smbus.SMBus(1)
        self.channel_voltage = MCP3422(channel=0)  # voltage
        self.channel_current = MCP3422(channel=1)  # current

        # Convertion ratios taken from PSM datasheet at: https://bluerobotics.com/store/comm-control-power/control/psm-asm-r2-rp/
        self.psm_to_battery_voltage = 11.0  # V/V
        self.psm_to_battery_current_scale_factor = 37.8788  # A/V
        self.psm_to_battery_current_offset = 0.330  # V

        # Local variables
        self.system_voltage = 0.0
        self.system_current = 0.0
        
    def print_voltage_test(self):
        self._read_voltage()
        self._read_PSM_current()
        
        print(f">Voltage : {self.system_voltage} V")
        print(f">Current : {self.system_current} A")
        print()

    def _read_voltage(self):
        # Sometimes an I/O timeout or error happens, it will run again when the error disappears
        try:
            self.system_voltage = (
                self.channel_voltage.get_voltage_from_reading()
                * self.psm_to_battery_voltage
            )


        except IOError:
            self.I2C_error_counter_voltage += 1

    def _read_PSM_current(self):
        try:
            self.system_current = (
                self.channel_current.get_voltage_from_reading()
                + self.psm_to_battery_current_offset
            ) * self.psm_to_battery_current_scale_factor


        except IOError:
            self.I2C_error_counter_current += 1

    def shutdown(self):
        self.system_timer.shutdown()
        self.log_timer.shutdown()
        self.bus.close()


if __name__ == "__main__":
    bm = BatteryMonitor()
    
    while True:
        bm.print_voltage_test()
        time.sleep(1)
   
