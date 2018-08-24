# -*- coding: utf-8 -*-
"""
Created on Thu Aug 23 02:50:34 2018

@author: Tyler
"""


import time
#import numpy as np
import matplotlib.pyplot as plt
#from mpl_toolkits.mplot3d import axes3d


from SurfaceRobot import SurfaceRobot
from RobotModel import Robot
from APF import APF



R = Robot([40,0,0],[2,2,-0.5,45,10],0.5,[2,2,2,4,4])
#(self,InitState,SensInput,TimeStep,SensVar)
S = SurfaceRobot(-50,0,90,0.5,0.5)
#(self,X,Y,Velocity,Angle,dt)
A = APF(S.state[0,0],S.state[1,0], 15)
#(self,Sx,Sy,RobotRadius)
R.PosLog()
S.Log()

AN = 100

start_time = time.time()

R.InputUpdate(1.5,1.5,-1.5,AN,-20)



plt.plot(-20,0,'ro')
plt.plot(22,18,'ro')
plt.plot(30,55,'ro')
plt.plot(27,50,'ro')
plt.plot(30,30,'ro')
plt.plot(0,5,'ro')
plt.plot(20,0,'ro')
plt.plot(20,10,'ro')
plt.plot(10,18,'ro')
plt.plot(20,30,'ro')
plt.plot(20,40,'ro')
plt.plot(20,80,'ro')
plt.plot(19,75,'ro')
plt.plot(11,75,'ro')
plt.plot(21,60,'ro')



for i in range(0,360):
    R.InputUpdate(0.5,0,-1.5,AN,-20)
    R.Movement()
    R.Odometry()
        
    R.rangecalc(S.state[0,0],S.state[1,0],0)
       
        
        
    A.GradCalcGoal(R.state[0,0],R.state[1,0],1,1.5)
    A.GradCalcOb (15,[20,19,11,21,-20,27,30,22,30,0,20,20,10,20,20], [80,75,75,60,0,50,55,18,30,5,0,10,18,30,40], [3,4,3,6,5,4,4,3,3,6,4,4,4,4,4], [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1], [1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5])
    A.CalcOutput()
    S.InputUpdate(A.Velocity,A.Angle)
    S.Move()
    A.StateUpdate(S.state[0,0],S.state[1,0])
    
    R.PosLog()
    S.Log()
    """
    if i % 200 == 0:
        AN = 45
    """
    
for i in range(0,200):
    R.InputUpdate(0.5,0,-1.5,-175,-20)
    R.Movement()
    R.Odometry()
        
    R.rangecalc(S.state[0,0],S.state[1,0],0)
       
        
        
    A.GradCalcGoal(R.state[0,0],R.state[1,0],1,1.5)
    A.GradCalcOb (15,[20,19,11,21,-20,27,30,22,30,0,20,20,10,20,20], [80,75,75,60,0,50,55,18,30,5,0,10,18,30,40], [3,4,3,6,5,4,4,3,3,6,4,4,4,4,4], [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1], [1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5])

    A.CalcOutput()
    S.InputUpdate(A.Velocity,A.Angle)
    S.Move()
    A.StateUpdate(S.state[0,0],S.state[1,0])
    
    R.PosLog()
    S.Log()

plt.plot(R.GroundTruthx,R.GroundTruthy,'g:',linewidth = 1, markersize = 1.5,label = 'Ground Truth')
plt.plot(S.logx,S.logy,'m--',linewidth = 1, markersize = 1,label = 'Surface Path (APF)')
plt.xlabel('East-West (Metres)')
plt.ylabel('North-South (Metres)')
plt.tight_layout()
plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)

plt.show()

timing = (time.time() - start_time)
print("--- %s seconds ---" % (time.time() - start_time))
print(len(R.GroundTruthy))