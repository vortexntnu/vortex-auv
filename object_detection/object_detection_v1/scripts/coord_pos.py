#!/usr/bin/env python


# import Locally used packages
from math import cos, sin, radians

class CoordPositionClass():
    def main(self):
        pass

    def calc_3D_vector(self, x_angle, y_angle, length):
        x_comp = length * cos(radians(x_angle)) * sin(radians(y_angle))
        y_comp = length * cos(radians(x_angle)) * cos(radians(y_angle))
        z_comp = length * sin(radians(x_angle))

        return [x_comp, y_comp, z_comp]