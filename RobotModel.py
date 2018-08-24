# -*- coding: utf-8 -*-
"""
Created on Sat Aug  4 15:17:06 2018

@author: Tyler
"""

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d


class Robot:
    def __init__(self,InitState,SensInput,TimeStep,SensVar):
        #Position state
        self.state = np.empty((3,1))
        self.state[0,0] = InitState[0]
        self.state[1,0] = InitState[1]
        self.state[2,0] = InitState[2]
        #Odometry state
        self.Odostate = np.empty((3,1))
        self.Odostate[0,0] = InitState[0]
        self.Odostate[1,0] = InitState[1]
        self.Odostate[2,0] = InitState[2]
        #Inputs from sensors
        self.input = np.empty((5,1))
        self.input[0,0] = SensInput[0] #Xv
        self.input[1,0] = SensInput[1] #Yv
        self.input[2,0] = SensInput[2] #Zv
        self.input[3,0] = np.radians(SensInput[3]) #Bearing Angle
        self.input[4,0] = np.radians(SensInput[4]) #Pitch Angle
        #Range to surface vehicle
        self.range = 0
        #Time Step
        self.dt = TimeStep
        #Sensor Variance
        self.inVar = np.empty((5,1))
        self.inVar[0,0] = SensVar[0] #Xv Variance
        self.inVar[1,0] = SensVar[1] #Yv Variance
        self.inVar[2,0] = SensVar[2] #Zv Variance
        self.inVar[3,0] = SensVar[3] #Bearing Variance
        self.inVar[4,0] = SensVar[4] #Pitch Variance
        #Position Logs
        self.GroundTruthx = []
        self.GroundTruthy = []
        self.GroundTruthz = []
        
        self.OdoLogx = []
        self.OdoLogy = []
        self.OdoLogz = []
        
    def InputUpdate(self,Xv,Yv,Zv,Angle,Pitch):
        self.input[0,0] = Xv
        self.input[1,0] = Yv
        self.input[2,0] = Zv
        self.input[3,0] = np.radians(Angle)
        self.input[4,0] = np.radians(Pitch)
        
    def Movement(self):
        self.state[0,0] = self.state[0,0] + self.dt*((self.input[0,0]*np.cos(self.input[3,0]))-(self.input[1,0]*np.sin(self.input[3,0])))
        self.state[1,0] = self.state[1,0] + self.dt*((self.input[0,0]*np.sin(self.input[3,0]))+(self.input[1,0]*np.cos(self.input[3,0])))
        self.state[2,0] = self.state[2,0] + self.dt* ((self.input[0,0]*np.sin(self.input[4,0]))+(self.input[2,0]*np.cos(self.input[4,0])))
    
    def Odometry(self):
        #Define Inputs with variance
        Xv = self.input[0,0] + np.random.normal(0,self.inVar[0,0])
        Yv = self.input[1,0] + np.random.normal(0,self.inVar[1,0])
        Zv = self.input[2,0] + np.random.normal(0,self.inVar[2,0])
        bearing = self.input[3,0] + np.radians(np.random.normal(0,self.inVar[3,0]))
        pitch = self.input[4,0] + np.radians(np.random.normal(0,self.inVar[4,0]))
        
        #Odometry movement calculation
        self.Odostate[0,0] = self.Odostate[0,0] + self.dt*((Xv*np.cos(bearing))-(Yv*np.sin(bearing)))
        self.Odostate[1,0] = self.Odostate[1,0] + self.dt*((Xv*np.sin(bearing))+(Yv*np.cos(bearing)))
        self.Odostate[2,0] = self.Odostate[2,0] + self.dt* ((Xv*np.sin(pitch))+(Zv*np.cos(pitch)))
   
    def rangecalc(self,X2,Y2,Z2):
        self.range = np.sqrt((self.state[0,0] - X2)**2 +(self.state[1,0] - Y2)**2 + (self.state[2,0] - Z2)**2)
    
    def PosLog(self):
        #Ground truth logs
        self.GroundTruthx.append(self.state[0,0])
        self.GroundTruthy.append(self.state[1,0])
        self.GroundTruthz.append(self.state[2,0])
        #Odometry logs
        self.OdoLogx.append(self.Odostate[0,0])
        self.OdoLogy.append(self.Odostate[1,0])
        self.OdoLogz.append(self.Odostate[2,0])
"""

R = Robot([0,0,0],[2,2,-0.5,15,10],0.5,[0.5,0.5,0.5,1,1])
plt.plot(R.state[0,0],R.state[1,0],'b*')
R.PosLog()
for i in range(0,10):
    R.Movement()
    R.Odometry()
    R.PosLog()
    


R.InputUpdate(0,1.4,-0.3,0,30)

for i in range (0,30):
    R.Movement()
    R.Odometry()
    R.PosLog()
    

plt.plot(R.GroundTruthx,R.GroundTruthy,'r--',linewidth = 0.5, markersize = 5)
plt.plot(R.OdoLogx,R.OdoLogy,'g--',linewidth = 0.5, markersize = 5)

plt.show()
"""