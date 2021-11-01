#!/usr/bin/env python

from math import cos, sin, radians

class CoordPosition():

    def main(self, x_angle, y_angle, length):
        """
        Calculates the position of an object

        Args:
            x_angle: Angle to object in regards to x.
            y_angle: Angle to object in regards to y.
            length: Length to the object

        Returns:
            List: X-position, Y-position, Z-position
        """
        x_comp = length * cos(radians(x_angle)) * sin(radians(y_angle))
        y_comp = length * cos(radians(x_angle)) * cos(radians(y_angle))
        z_comp = length * sin(radians(x_angle))

        return [x_comp, y_comp, z_comp]