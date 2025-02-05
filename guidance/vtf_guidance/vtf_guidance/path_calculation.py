#!/usr/bin/python3
# Written by Aksel Kristoffersen
# Documentation can be found in Aksels master's thesis: Guidance and Control System for Dynamic Positioning and PathFollowing of an AUV exposed to Ocean Currents

import numpy as np


# function to calculate the smallest signed angle
def ssa(angle):
    if angle < -np.pi or angle >= np.pi:
        # Map angle to [-pi, pi)
        angle = (angle + np.pi) % (2 * np.pi) - np.pi
    return angle


# function to calculate the rotation matrix around the z-axis
def R_z(rot):
    return np.array(
        [[np.cos(rot), -np.sin(rot), 0], [np.sin(rot), np.cos(rot), 0], [0, 0, 1]]
    )


# class to generate a path
class Path:
    def __init__(self):
        self.path = []
        self.chi_p = []
        self.gamma_p = []
        self.kappa_h = []
        self.kappa_v = []
        self.length = []

    # path generation with only straight lines
    def generate_G0_path(self, waypoints):
        for i in range(len(waypoints) - 1):
            v = [a - b for a, b in zip(waypoints[i + 1], waypoints[i])]
            l = np.linalg.norm(v)
            chi_l = np.arctan2(v[1], v[0])
            gamma_l = -np.arctan2(v[2], np.linalg.norm(v[:2]))
            self.straight_line_path_segment(waypoints[i], l, chi_l, gamma_l)

    # path generation with straight lines and curved arcs
    def generate_G1_path(self, waypoints, r_h, r_v):
        wp_0 = waypoints[0]
        for i in range(len(waypoints) - 2):
            v1 = [a - b for a, b in zip(waypoints[i + 1], wp_0)]
            v2 = [a - b for a, b in zip(waypoints[i + 2], waypoints[i + 1])]

            chi_0 = np.arctan2(v1[1], v1[0])
            gamma_0 = -np.arctan2(v1[2], np.linalg.norm(v1[:2]))
            chi_1 = np.arctan2(v2[1], v2[0])
            gamma_1 = -np.arctan2(v2[2], np.linalg.norm(v2[:2]))
            """
            #alpha_h = -(np.arctan2(v1[0]*v2[1]-v1[1]*v2[0], v1[0]*v2[0]-v1[1]*v2[1]))/2
            alpha_h = (chi_1-chi_0+np.pi)/2
            alpha_v = (gamma_1-gamma_0+np.pi)/2
            v1_v = v1[2]*np.linalg.norm(v1[:2])
            #v2_v = v2[2]*np.linalg.norm(v2[:2])
            #alpha_v = (np.arctan2(v1_v[0]*v2_v[1]-v1_v[1]*v2_v[0], v1_v[0]*v2_v[0]-v1_v[1]*v2_v[1]))/2
            cut_h = r_h / np.tan(alpha_h)
            cut_v = r_v / np.tan(alpha_v)
            tot_cut = np.sqrt(cut_h**2 + cut_v**2)
            l_1 = np.linalg.norm(v1)-tot_cut
            """
            # alpha_h = -(np.arctan2(v1[0]*v2[1]-v1[1]*v2[0], v1[0]*v2[0]-v1[1]*v2[1]))/2
            alpha_h = (chi_1 - chi_0 + np.pi) / 2
            # v2_v = v2[2]*np.linalg.norm(v2[:2])
            # alpha_v = (np.arctan2(v1_v[0]*v2_v[1]-v1_v[1]*v2_v[0], v1_v[0]*v2_v[0]-v1_v[1]*v2_v[1]))/2
            cut_h = r_h / np.tan(alpha_h)
            tot_cut = np.sqrt(cut_h**2 + (cut_h * np.sin(gamma_0)) ** 2)
            l_1 = np.linalg.norm(v1) - tot_cut

            self.straight_line_path_segment(wp_0, l_1, chi_0, gamma_0)

            wp_1 = self.path[-1](1)
            c = wp_1 + np.sign(ssa(chi_1 - chi_0)) * np.dot(R_z(chi_0), [0, r_h, 0])
            c_v = wp_1[2] - np.sign(gamma_1 - gamma_0) * np.cos(gamma_0) * r_v
            c[2] = waypoints[i + 1][2]
            # print(c[0]+np.sin(chi_1)*r_h)
            # print(c[1]-np.cos(chi_1)*r_h)
            # print(c)

            self.curved_path_segment(c, r_h, r_v, chi_0, chi_1, gamma_0, gamma_1)

            if i == len(waypoints) - 2 - 1:
                wp_2 = self.path[-1](1)
                l_2 = np.linalg.norm(v2) - tot_cut
                self.straight_line_path_segment(wp_2, l_2, chi_1, gamma_1)
            wp_0 = self.path[-1](1)

    def straight_line_path_segment(self, p_0, l, chi_l, gamma_l):
        self.path.append(
            lambda varpi: [
                p_0[0] + l * varpi * np.cos(chi_l) * np.cos(gamma_l),
                p_0[1] + l * varpi * np.sin(chi_l) * np.cos(gamma_l),
                p_0[2] - l * varpi * np.sin(gamma_l),
            ]
        )
        self.chi_p.append(lambda varpi: chi_l)
        self.gamma_p.append(lambda varpi: gamma_l)
        self.kappa_h.append(lambda varpi: 0)
        self.kappa_v.append(lambda varpi: 0)
        self.length.append(l)

    def curved_path_segment(self, c, r_h, r_v, chi_0, chi_1, gamma_0, gamma_1):
        l_1 = abs(r_h * ssa(chi_1 - chi_0) * np.sin(gamma_0)) / 2
        l_2 = abs(r_h * ssa(chi_1 - chi_0) * np.sin(gamma_1)) / 2
        self.path.append(
            lambda varpi: [
                c[0]
                - np.sign(ssa(chi_0 - chi_1))
                * r_h
                * np.sin(chi_0 + varpi * (ssa(chi_1 - chi_0))),
                c[1]
                + np.sign(ssa(chi_0 - chi_1))
                * r_h
                * np.cos(chi_0 + varpi * (ssa(chi_1 - chi_0))),
                c[2]
                + (
                    -np.sign(gamma_0) * l_1 * (varpi - 0.5) * 2
                    if varpi < 0.5
                    else -np.sign(gamma_1) * l_2 * (varpi - 0.5) * 2
                ),
            ]
        )
        self.chi_p.append(lambda varpi: ssa(chi_0 + varpi * (ssa(chi_1 - chi_0))))
        self.gamma_p.append(lambda varpi: (gamma_0 if varpi < 0.5 else gamma_1))
        self.kappa_h.append(lambda varpi: np.sign(ssa(chi_1 - chi_0)) * 1 / r_h)
        self.kappa_v.append(lambda varpi: 0)
        length = np.sqrt((abs(r_h * ssa(chi_1 - chi_0)) / 2) ** 2 + l_1**2) + np.sqrt(
            (abs(r_h * ssa(chi_1 - chi_0)) / 2) ** 2 + l_2**2
        )
        self.length.append(length)
