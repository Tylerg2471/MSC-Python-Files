# -*- coding: utf-8 -*-
"""
Created on Mon Jul 23 21:57:02 2018

@author: Tyler
"""

import numpy as np
import matplotlib
import matplotlib.pyplot as plt

class SurfaceRobot:
    def __init__(self,X,Y,Velocity,Angle,dt):
        #Initial Position
        self.state  = np.empty((2,1))
        self.state[0,0] = X
        self.state[1,0] = Y
        #Initial Position
        self.input = np.empty((2,1))
        self.input[0,0] = Velocity
        self.input[1,0] = np.radians(Angle)
        #Time Step
        self.dt = dt
        #Signal Sending
        self.T = 0
        self.SigSend = 0
        #Logs for position
        self.logx = []
        self.logy = []
        
    
    def InputUpdate(self,Velocity,Angle):
        self.input[0,0] = Velocity
        self.input[1,0] = np.radians(Angle)
        
    def Move(self):
        
        self.state[0,0] = self.state[0,0] + self.dt*(self.input[0,0] * np.cos(self.input[1,0]))
        self.state[1,0] = self.state[1,0] + self.dt* (self.input[0,0] * np.sin(self.input[1,0]))
        
    
    def Log(self):
        self.logx.append(self.state[0,0])
        self.logy.append(self.state[1,0])
        

        


     
        
 