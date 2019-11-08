#!/usr/bin/env python
# Written by Kristoffer Rakstad Solberg, Student
# Copyright (c) 2020 Manta AUV, Vortex NTNU.
# All rights reserved.

import numpy as np

class BacksteppingDesign:

    def __init__(self):
        
        # numerical values
        
        # model parameters
        # acceleration proportional
        self.m      =  50.000
        self.I_z    =  1.9198
        self.x_g    =  0.0000
        self.Xu_dot = -20.7727
        self.Yv_dot = -20.7727
        self.Nv_dot =  0.0000
        self.Yr_dot =  0.0000
        self.Nr_dot =  0.0000
        
        # velocity proportional
        self.Xu = -19.5909
        self.Yv = -19.5909
        self.Nv =  0.0000
        self.Yr =  0.0000
        self.Nr = -1.1559
        
        # static matrices
        
        self.M = np.array(( (self.m - self.Xu_dot, 0.0, 0.0),
                            (0.0, self.m - self.Yv_dot, self.m*self.x_g - self.Yr_dot),
                            (0.0, self.m*self.x_g - self.Nv_dot, self.I_z - self.Nr_dot) ))
        print(self.M)
        # nonlinear dynamics vector
        self.n = np.identity(3)
        
    def rotationMatrix(self, psi):
        
        R = np.array(( (np.cos(psi), -np.sin(psi), 0),
                       (np.sin(psi),  np.cos(psi), 0),
                       (          0,            0, 1) ))
        
    def nonlinVector(self, u):

        self.n = np.array(( (-self.Xu, 0.0,          0.0),
                            (0.0, -self.Yv,       self.m*u - self.Yr),
                            (0.0, -self.Nv, self.m*self.x_g*u - self.Nr) ))


class BacksteppingControl:

    def __init__(self, c, k1, k2, k3):
        
        # backstepping init
        self.bs = BacksteppingDesign()
        
        self.c = c #heading gain
        
        self.K = np.array(( (k1, 0, 0), # surge speed gain
                            (0, k2, 0), # sway speed gain  
                            (0, 0, k3) )) # 
        
        self.nu = np.transpose(np.array((0,0,0)))
        self.h = np.transpose(np.array((0,0,1)))
        self.alpha = np.transpose(np.array((0,0,0)))
        self.alpha_dot = np.transpose(np.array((0,0,0)))
        
    
    def updateState(self, u, u_dot, v, psi, r):
        
        # remains constant
        self.psi = psi
        self.r = r
        self.bs.nonlinVector(u)
        self.u_dot = u_dot
        self.nu = np.transpose(np.array((u, v, r)))
    
    def updateSetpoint(self, u_d, u_d_dot, psi_d, r_d, r_d_dot):
    
        # in ENU
        self.z1 = (self.psi - psi_d)
        self.z2 = self.nu - self.alpha
    
        alpha_1 = u_d #stabilizing function
        alpha_2 = 0.0
        alpha_3 = -self.c*self.z1 + r_d #virtual control
        self.alpha = np.transpose(np.array((alpha_1, alpha_2, alpha_3)))
        
        
        alpha_1_dot = u_d_dot
        alpha_2_dot = 0 # dissapears
        alpha_3_dot = -self.c*(self.r-r_d) + r_d_dot
        self.alpha_dot = np.transpose(np.array((alpha_1_dot, alpha_2_dot, alpha_3_dot)))
        
    
    def controlLaw(self, u, u_dot, u_d, u_d_dot, v, psi, psi_d, r, r_d, r_d_dot):
            
        # update state
        self.updateState(u, u_dot, v, psi, r)
        
        # update setpoint
        self.updateSetpoint(u_d, u_d_dot, psi_d, r_d, r_d_dot)
  
        # control force
        tau = self.bs.M.dot(self.alpha_dot) + self.bs.n.dot(self.nu) - self.K.dot(self.z2) - self.h.dot(self.z1)
        a = self.bs.n.dot(self.nu) 
        b = self.K.dot(self.z2)
        #print(tau)
        
        return tau
