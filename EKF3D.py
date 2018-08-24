# -*- coding: utf-8 -*-
"""
Created on Sat Aug  4 16:42:56 2018

@author: Tyler
"""
from RobotModel import Robot
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

class EKF_3D:
    def __init__(self,Rstate,RInput,ProcessVar,MeasurementVar,InputVar,StateVar,DepthSensVar,dt):
        #Initial State of the System
        self.state = np.empty((3,1))
        self.state[0,0] = Rstate[0]
        self.state [1,0] = Rstate[1]
        self.state[2,0] = Rstate[2]
        
        #Initial Input values to system of X velocity, Y velocity and Bearing
        self.input = np.empty((5,1))
        self.input[0,0] = RInput[0] #Xv
        self.input[1,0] = RInput[1] #Yv
        self.input[2,0] = RInput[2] #Zv
        self.input[3,0] = RInput[3] #Angle
        self.input[4,0] = RInput[4] #Pitch
        
        #Surface State
        self.surf = np.zeros((2,1))
        
        #Noise for the process. In this case it is assumed no corealation noise
        self.Q = np.zeros((3,3))
        self.Q[0,0] = ProcessVar[0] **2
        self.Q[1,1] = ProcessVar[1] **2 
        self.Q[2,2] = ProcessVar[2] **2
        
        #Variance for the measurement in this case the range
        self.R = MeasurementVar **2
        
        #Variance for the inputs 
        self.InVar = np.zeros((5,5))
        self.InVar[0,0] = InputVar[0] **2
        self.InVar[1,1] = InputVar[1] **2
        self.InVar[2,2] = InputVar[2] **2
        self.InVar[3,3] = InputVar[3] **2
        self.InVar[4,4] = InputVar[4] **2
        
        #Variance for the state
        self.P = np.zeros((3,3))
        self.P[0,0] = StateVar[0] **2
        self.P[1,1] = StateVar[1] **2
        self.P[2,2] = StateVar[2] **2
        
        #Variance for depth sensor
        self.DepthVar = DepthSensVar
        
        self.dt = dt
        
        #B Jacobian
        self.B = np.zeros((3,5))
        
        #H Jacobian
        self.H = np.zeros((1,3))
        
        #Logs for position
        self.logx =[]
        self.logy =[]
        self.logz =[]
        
    def InputUpdate(self,RInput):
        self.input[0,0] = RInput[0] #X
        self.input[1,0] = RInput[1] #Y
        self.input[2,0] = RInput[2] #Z
        self.input[3,0] = RInput[3] #Angle
        self.input[4,0] = RInput[4] #Pitch      
        
    def predict(self):
        self.state[0,0] = self.state[0,0] + self.dt*((self.input[0,0]*np.cos(self.input[3,0]))-(self.input[1,0]*np.sin(self.input[3,0]))) #+ np.random.normal(0,self.Q[0,0])
        self.state[1,0] = self.state[1,0] + self.dt*((self.input[0,0]*np.sin(self.input[3,0]))+(self.input[1,0]*np.cos(self.input[3,0]))) #+ np.random.normal(0,self.Q[1,1])
        self.state[2,0] = self.state[2,0] + self.dt* ((self.input[0,0]*np.sin(self.input[4,0]))+(self.input[2,0]*np.cos(self.input[4,0]))) #+ np.random.normal(0,self.Q[2,2])
        
        #Calculate B Jacobian 
        self.B[0,0] = -np.sin(self.input[3,0]) #-sin(Angle)
        self.B[1,0] = np.cos(self.input[3,0]) #cos(Angle)
        self.B[2,0] = np.cos(self.input[4,0]) #cos(Pitch Angle)
        self.B[0,1] = np.cos(self.input[3,0]) #cos(Angle)
        self.B[1,1] = np.sin(self.input[3,0]) #sin(Angle)
        self.B[2,1] = 0
        self.B[0,2] = 0
        self.B[1,2] = 0
        self.B[2,2] = -np.sin(self.input[4,0])
        self.B[0,3] = -((self.input[1,0] * np.sin(self.input[3,0])) - (self.input[0,0] * np.cos(self.input[3,0]))) # -Xv * sin(Angle) + Yv * cos(Angle)
        self.B[1,3]  = (self.input[1,0] * np.cos(self.input[3,0]) - self.input[0,0] * np.sin(self.input[3,0])) #Xv * cos(Angle) - Yv * sin(Angle)
        self.B[2,3] = 0
        self.B[0,4] = 0
        self.B[1,4] = 0
        self.B[2,4] = (self.input[2,0] * np.cos(self.input[4,0]) - self.input[0,0] * np.sin(self.input[4,0])) #Xv * cos(Pitch) - Zv * sin(Pitch)
        
        #B*InVar
        Temp = np.dot(self.B,self.InVar)
        BT = np.transpose(self.B)
        Temp2 = np.dot(Temp,BT)
        
        #P't = F*Pt-1*FT + B*InVar t-1 * BT + Q       
        
        self.P = self.P + Temp2 + self.Q
        
    def KGain(self,SurfaceX,SurfaceY,Depth):
        #Assign Surface Values
        self.surf[0,0] = SurfaceX
        self.surf[1,0] = SurfaceY
        self.depthsens = Depth + np.random.normal(0,self.DepthVar)
        #Measurement Model
        self.hX = np.sqrt((self.state[0,0] - self.surf[0,0])**2 + (self.state[1,0] - self.surf[1,0])**2 + (self.depthsens)**2) + np.random.normal(0,self.R)
         
        
        #H Jacobian
        self.H[0,0] = (self.state[0,0]-self.surf[0,0]) / (np.sqrt((self.state[0,0]-self.surf[0,0])**2 + (self.state[1,0] - self.surf[1,0])**2 + (self.depthsens)**2))
        self.H[0,1] = (self.state[1,0]-self.surf[1,0]) / (np.sqrt((self.state[0,0]-self.surf[0,0])**2 + (self.state[1,0] - self.surf[1,0])**2 + (self.depthsens)**2))
        self.H[0,2] = (self.state[2,0]) / (np.sqrt((self.state[0,0]-self.surf[0,0])**2 + (self.state[1,0] - self.surf[1,0])**2 + (self.depthsens)**2))
        
        #H Transpose
        self.HT = np.transpose(self.H)
        
        #K = P*HT * (H * P * HT + R)^-1
        TempPHT = np.dot(self.P,self.HT)
        TempHP = np.dot(self.H,self.P)
        TempHPHT = np.dot(TempHP,self.HT) + self.R
        Tempinv = np.linalg.inv(TempHPHT)
        self.K =  np.dot(TempPHT,Tempinv)

    def update(self,Tsend,Trec,Speed):
         
         #Observation Model
         self.Obsv = Speed * (Trec-Tsend) + np.random.normal(0,self.R)
         
         #Residual
         self.Z = self.Obsv - self.hX
         
         #Updated state Xt = Xt + K(Zt)
         self.state = self.state + np.dot(self.K, self.Z)
    
    def RUN(self,SurfaceX,SurfaceY,Depth,Speed):
        self.predict()
        self.KGain(SurfaceX,SurfaceY,Depth)
        TimeDist = self.hX/Speed
        self.update(0,TimeDist,Speed)
        
    def Log(self):
        self.logx.append(self.state[0,0])
        self.logy.append(self.state[1,0])
        self.logz.append(self.state[2,0])
"""        
R = Robot([10,10,0],[2.3,0.4,-1,10,-30],0.5,[7,7,7,8,8])
#(self,InitState,SensInput,TimeStep,SensVar)
EKF = EKF_3D(R.state,R.input,[0.5,0.5,0.5],0.5,R.inVar,[1,1,1],R.dt)
#(self,Rstate,RInput,ProcessVar,MeasurementVar,InputVar,StateVar,dt)

R.PosLog()
EKF.Log()

for i in range(0,100):
    
    R.Movement()
    R.Odometry()
    R.rangecalc(25,25,0)
    
    EKF.predict()
    EKF.KGain(25,25,R.state[2,0])

    TimeDist = EKF.hX/1500
    EKF.update(0,TimeDist,1500)
    
    R.PosLog()
    EKF.Log()
 
R.InputUpdate(1.5,1.5,-1.5,12,-20)
EKF.InputUpdate(R.input)

for i in range(0,300):
    
    R.Movement()
    R.Odometry()
    R.rangecalc(25,25,0)
    
    EKF.predict()
    EKF.KGain(25,25,R.state[2,0])
    
    TimeDist = EKF.hX/1500
    EKF.update(0,TimeDist,1500)
    
    
    R.PosLog()
    EKF.Log()

R.InputUpdate(1,0.8,0,-90,0)
EKF.InputUpdate(R.input)

for i in range(0,100):
    
    R.Movement()
    R.Odometry()
    R.rangecalc(25,25,0)
    
    EKF.predict()
    EKF.KGain(25,25,R.state[2,0])
    
    TimeDist = EKF.hX/1500
    EKF.update(0,TimeDist,1500)
    
    
    R.PosLog()
    EKF.Log()

R.InputUpdate(1.2,1.4,0.4,-54,0)
EKF.InputUpdate(R.input)

for i in range(0,50):
    
    R.Movement()
    R.Odometry()
    R.rangecalc(25,25,0)
    
    EKF.predict()
    EKF.KGain(25,25,R.state[2,0])
    
    TimeDist = EKF.hX/1500
    EKF.update(0,TimeDist,1500)
    
    
    R.PosLog()
    EKF.Log()


 

ekferrx = (R.state[0,0] - EKF.state[0,0])
ekferry = (R.state[1,0] - EKF.state[1,0])
ekferrz = (R.state[2,0] - EKF.state[2,0])

odoerrx =(R.state[0,0] - R.Odostate[0,0])
odoerry =(R.state[1,0] - R.Odostate[1,0])
odoerrz =(R.state[2,0] - R.Odostate[2,0])

plt.plot(R.GroundTruthx,R.GroundTruthy,'g--',linewidth = 1, markersize = 5)
plt.plot(R.OdoLogx,R.OdoLogy,'r--',linewidth = 1, markersize = 5)
plt.plot(EKF.logx,EKF.logy,'b--',linewidth = 1, markersize = 5)

plt.show()
print("EKF Error  =",ekferrx,ekferry,ekferrz)
print("Odo Error  =",odoerrx,odoerry,odoerrz)
print((R.state[0,0]/EKF.state[0,0]),(R.state[1,0]/EKF.state[1,0]),(R.state[2,0]/EKF.state[2,0]))
"""