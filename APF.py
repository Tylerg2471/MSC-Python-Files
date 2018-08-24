# -*- coding: utf-8 -*-
"""
Created on Sat Aug  4 00:45:53 2018

@author: Tyler
"""

import numpy as np
import matplotlib
import matplotlib.pyplot as plt

class SurfaceRobot:
    def __init__(self,X,Y):
        self.state  = np.empty((2,1))
        self.state[0,0] = X
        self.state[1,0] = Y
        
        self.input = np.empty((2,1))
        
        self.T = 0
        self.SigSend = 0
        
    def Move(self,Velocity,Angle,dt):
        self.input[0,0] = Velocity
        self.input[1,0] = Angle
        
        self.state[0,0] = self.state[0,0] + dt*(self.input[0,0] * np.cos(self.input[1,0]))
        self.state[1,0] = self.state[1,0] + dt* (self.input[0,0] * np.sin(self.input[1,0]))

class APF:
    def __init__(self,Sx,Sy,RobotRadius):
        self.surf = np.empty((2,1))
        self.surf[0,0] = Sx
        self.surf[1,0] = Sy
        self.Rr = RobotRadius
        self.Xgrad = 0
        self.Ygrad = 0
        #obstacle gradients
        self.OXgrad = 0
        self.OYgrad = 0
        #Total Gradients
        self.TXgrad = 0
        self.TYgrad = 0
        self.OsumX = 0
        self.OsumY = 0
        
    def StateUpdate(self,Sx,Sy):
        self.surf[0,0] = Sx
        self.surf[1,0] = Sy
    
    def GradCalcGoal(self,Rx,Ry,ScaleFact,Spread):
        dist = np.sqrt((Rx - self.surf[0,0])**2 + (Ry - self.surf[1,0])**2)
        Temp = (Ry - self.surf[1,0])/(Rx - self.surf[0,0])
        angle = np.arctan2((Ry - self.surf[1,0]),(Rx - self.surf[0,0])) #Changed to ARCTAN2
        Sf = ScaleFact
        s = Spread
        
        if dist < self.Rr:
            self.Xgrad = 0
            self.Ygrad = 0
            
        elif self.Rr <= dist and dist <= (s + self.Rr):
            self.Xgrad = Sf * ((dist - self.Rr) * np.cos(angle))
            self.Ygrad = Sf * ((dist-self.Rr) * np.sin(angle))
            
        elif  dist > (s + self.Rr):
            self.Xgrad = Sf *(s * np.cos(angle))
            self.Ygrad = Sf *(s * np.sin(angle))
    
    def GradCalcOb (self,NObs,ObsX, ObsY, ObsR, ScaleFact, Spread):
        self.OsumX = 0
        self.OsumY = 0
        
        self.Obs = np.empty((NObs,5))
        #set the obstacles up
        for i in range(0,(NObs)):
            self.Obs[i,0] = ObsX[i]
            self.Obs[i,1] = ObsY[i]
            self.Obs[i,2] = ObsR[i]
            self.Obs[i,3] = ScaleFact[i]
            self.Obs[i,4] = Spread[i]
        
        for i in range (0,(NObs)):
            Odist = np.sqrt((self.Obs[i,0] - self.surf[0,0])**2 + (self.Obs[i,1] - self.surf[1,0])**2)
            OTemp = (self.Obs[i,1] - self.surf[1,0])/(self.Obs[i,0] - self.surf[0,0])
        
            Oangle = np.arctan2((self.Obs[i,1] - self.surf[1,0]),(self.Obs[i,0] - self.surf[0,0]))
            OSf = self.Obs[i,3]
            Inf = 100000
            Os = self.Obs[i,4]
        
            if Odist < self.Obs[i,2]:
                self.OXgrad = -(np.sign(np.cos(Oangle) * Inf))
                self.OYgrad = -(np.sign(np.sin(Oangle) * Inf))
            
            elif self.Obs[i,2] <= Odist and Odist <= (Os + self.Obs[i,2]):
                self.OXgrad = -OSf * ((Os + self.Obs[i,2] - Odist) * np.cos(Oangle))
                self.OYgrad = -OSf * ((Os + self.Obs[i,2] - Odist) * np.sin(Oangle))
            
            elif  Odist > (Os + self.Obs[i,2]):
                self.OXgrad = 0
                self.OYgrad = 0
            
            
            self.OsumX += self.OXgrad
            self.OsumY += self.OYgrad
            
    def CalcOutput(self):
        Atemp = 0
        
        self.Angle = 0
        self.TXgrad = self.Xgrad + self.OsumX 
        self.TYgrad = self.Ygrad + self.OsumY 
        #Set TXgrad and TYgrad to minimal value to avoid division by 0
        if self.TXgrad == 0:
            self.TXgrad = 0.0000001
        if self.TYgrad == 0:
            self.TYgrad = 0.0000001
        #self.TXgrad = np.round(self.TXgrad,decimals = 2)
        #self.TYgrad = np.round(self.TYgrad,decimals = 2)
        
        self.Velocity = np.sqrt((self.TXgrad)**2 + (self.TYgrad)**2)
        Atemp = (self.TYgrad/self.TXgrad)
        self.Angle = np.rad2deg(np.round(np.arctan2(self.TYgrad,self.TXgrad),decimals = 2))
        


"""
            

S = SurfaceRobot(0,10)      
A = APF(S.state[0,0],S.state[1,0], 0.2)

plt.plot(30,0,'ko',markersize = 10)
plt.plot(10,6,'co',markersize = 10)
plt.plot(17,5,'ro',markersize =10)
#plt.plot(24,3,'ro',markersize =10)
plt.plot(S.state[0,0],S.state[1,0],'g*')    

logx = []
logy = []

for i in range(0,200):
    
    A.GradCalcGoal(30,0,1,)
    A.GradCalcOb(2,[10,17],[6,5],[1,1], [1,1],[1,1]) 
    A.CalcOutput()
    
    
    #print((np.rad2deg(A.Angle)), A.Velocity, A.TXgrad, A.TYgrad)
    
    S.Move(A.Velocity,A.Angle,1)
    A.StateUpdate(S.state[0,0],S.state[1,0])
    logx.append(S.state[0,0])
    logy.append(S.state[1,0])
    

    
    #plt.plot(S.state[0,0],S.state[1,0],'b*')
    #plt.plot(S.state[0,0],S.state[1,0],'go--', linewidth=0.5, markersize=3)

    

smoothx = []
smoothy = []
for i in range (0,100):
    if i == 0:
        Sx = (logx[i] + logx[i+1] + logx[i+2])/3
        Sy =(logy[i] + logy[i+1] + logy[i+2])/3
    elif i > 1 and i < 98:
        Sx = (logx[i-2] + logx[i-1]+logx[i] + logx[i+1] + logx[i+2])/5
        Sy =(logy[i-2] + logy[i-1]+ logy[i] +logy[i+1] + logy[i+2])/5
    elif i == 100:
        Sx =(logx[i] + logx[i-1] + logy[i-2])/3
        Sy = (logy[i] + logy[i-1] + logy[i-2])/3
    smoothx.append(Sx)
    smoothy.append(Sy)    

plt.plot(smoothx,smoothy,'b--', linewidth=0.5, markersize=3)

plt.show()
"""



  