# This file runs the code for the MCP3422 sensor
import board
import busio

i2c = busio.I2C(board.SCL, board.SDA)


class MCP3422:
    RESOLUTION_BITS = 0b11  # Resolution = 18 bits
    CHANNEL_ARRAY = {0: 0b00, 1: 0b01}
    GAIN_BITS = 0b00  # Gain == 1
    ADDRESS = 0x69

    def __init__(self, channel):
        self.channel = channel

    def _set_adc_channel(self):
        while not i2c.try_lock():
            pass

        # Wizardry from: https://github.com/benlhy/MCP3422/blob/master/mcp3422.py
        i2c.writeto(
            MCP3422.ADDRESS,
            bytes(
                [
                    0b1 << 7
                    | MCP3422.CHANNEL_ARRAY[self.channel] << 5
                    | 0b1 << 4
                    | MCP3422.RESOLUTION_BITS << 2
                    | MCP3422.GAIN_BITS
                ]
            ),
        )

    def _read(self):
        number = 0
        result = bytearray(3)
        i2c.readfrom_into(MCP3422.ADDRESS, result)
        number = (result[0] & 0b1) << 16 | result[1] << 8 | result[2]
        if result[0] & 0b10 == 1:
            number = -1 * number
        number = number * 15.625
        return number

    def get_voltage_from_reading(self) -> float:
        """get the analog voltage from reading
        Returns:
            float: the analog voltage in Volts
        """
        self._set_adc_channel()
        data = self._read()
        return data / 1000000.0
