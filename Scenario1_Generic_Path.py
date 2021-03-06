# -*- coding: utf-8 -*-
"""
Created on Sat Aug 11 18:31:56 2018

@author: Tyler
"""
import time
#import numpy as np
import matplotlib.pyplot as plt
#from mpl_toolkits.mplot3d import axes3d

from ParticleFilter import PF
from SurfaceRobot import SurfaceRobot
from RobotModel import Robot
from APF import APF
from EKF3D import EKF_3D

Errfile = open("Scen1P2000Err.txt","a+")



R = Robot([10,10,0],[2,2,-0.5,45,10],0.5,[2,2,2,4,4])
#(self,InitState,SensInput,TimeStep,SensVar)
PF = PF([-50,50],[-50,50],[-50,50],[0,359],2000,"G",[10,10,0,0],20)
#(self,Xrange,Yrange,Zrange,Anglerange,Amount,Distribution,GaussMean,GaussianVariance)
S = SurfaceRobot(-50,50,2,90,0.5)
#SA = SurfaceRobot(-50,50,2,90,0.5)
#SB = SurfaceRobot(-50,50,2,90,0.5)
#(self,X,Y,Velocity,Angle,dt)
A = APF(S.state[0,0],S.state[1,0], 15)
#AA = APF(S.state[0,0],S.state[1,0], 15)
#AB = APF(S.state[0,0],S.state[1,0], 15)
#(self,Sx,Sy,RobotRadius)
EKF = EKF_3D(R.state,R.input,[0.5,0.5,0.5],2,R.inVar,[1,1,1],R.dt,0.5)
#(self,Rstate,RInput,ProcessVar,MeasurementVar,InputVar,StateVar,dt)

PF.Input(R.input)
EKF.Log()
R.PosLog()
S.Log()
#SA.Log() 
#SB.Log() 

"""
#Plot initial distribution of particles
plt.figure(1)
plt.plot(PF.particles[:,0],PF.particles[:,1],'bo',markersize = 0.5,label = 'Initial Particles')
plt.figure(2)
plt.plot(PF.particles[:,0],PF.particles[:,2],'bo',markersize = 0.5,label = 'Initial Particles')
plt.figure(1)
"""
start_time = time.time()
for i in range(0,200):
    
    R.Movement()
    R.Odometry()
    
    R.rangecalc(S.state[0,0],S.state[1,0],0)
    PF.PFRUN(S.state[0,0],S.state[1,0],R.range,0.5,0.5)
    
    A.GradCalcGoal(EKF.state[0,0],EKF.state[1,0],1,1.5)
    A.CalcOutput()
    S.InputUpdate(A.Velocity,A.Angle)
    S.Move()
    A.StateUpdate(S.state[0,0],S.state[1,0])
    """
    AA.GradCalcGoal(PF.MX,PF.MY,1,1)
    AA.CalcOutput()
    SA.InputUpdate(AA.Velocity,AA.Angle)
    SA.Move()
    AA.StateUpdate(SA.state[0,0],SA.state[1,0])
    
    AB.GradCalcGoal(R.Odostate[0,0],R.Odostate[1,0],1,1.5)
    AB.CalcOutput()
    SB.InputUpdate(AB.Velocity,AB.Angle)
    SB.Move()
    AB.StateUpdate(SB.state[0,0],SB.state[1,0])
    """
    EKF.RUN(S.state[0,0],S.state[1,0],R.input[2,0],1500)
    
    R.PosLog()
    S.Log() 
    #SA.Log() 
    #SB.Log()     
    EKF.Log()
"""
plt.figure(1)
plt.plot(PF.particles[:,0],PF.particles[:,1],'bo',markersize = 0.5)
plt.figure(2)
plt.plot(PF.particles[:,0],PF.particles[:,2],'bo',markersize = 0.5)
plt.figure(1)
"""
R.InputUpdate(1.5,1.5,-1.5,12,-20)

S.InputUpdate(2,120)
#SB.InputUpdate(2,120)
#SA.InputUpdate(2,120)

PF.Input(R.input)
EKF.InputUpdate(R.input)

"""
plt.figure(1)
plt.plot(PF.particles[:,0],PF.particles[:,1],'go',markersize = 0.5,label = 'Converged Particles')
"""

for i in range(0,30):
    
    R.Movement()
    R.Odometry()
    
    A.GradCalcGoal(EKF.state[0,0],EKF.state[1,0],1,1.5)
    A.CalcOutput()
    S.InputUpdate(A.Velocity,A.Angle)
    S.Move()
    A.StateUpdate(S.state[0,0],S.state[1,0])
    """
    AA.GradCalcGoal(PF.MX,PF.MY,1,1)
    AA.CalcOutput()
    SA.InputUpdate(AA.Velocity,AA.Angle)
    SA.Move()
    AA.StateUpdate(SA.state[0,0],SA.state[1,0])
    
    AB.GradCalcGoal(R.Odostate[0,0],R.Odostate[1,0],1,1.5)
    AB.CalcOutput()
    SB.InputUpdate(AB.Velocity,AB.Angle)
    SB.Move()
    AB.StateUpdate(SB.state[0,0],SB.state[1,0])
    """
    R.rangecalc(S.state[0,0],S.state[1,0],0)
    
    PF.PFRUN(S.state[0,0],S.state[1,0],R.range,0.5,0.5)
    EKF.RUN(S.state[0,0],S.state[1,0],R.input[2,0],1500)
   
    R.PosLog()
    S.Log()
    #SA.Log() 
    #SB.Log()  
    EKF.Log()
"""
plt.figure(1)
plt.plot(PF.particles[:,0],PF.particles[:,1],'bo',markersize = 0.5)
plt.figure(2)
plt.plot(PF.particles[:,0],PF.particles[:,2],'bo',markersize = 0.5)
plt.figure(1)
"""
R.InputUpdate(1,0.8,0,-90,0)
S.InputUpdate(0.5,45)
PF.Input(R.input)
EKF.InputUpdate(R.input)

for i in range(0,100):
    
    R.Movement()
    R.Odometry()
    
    A.GradCalcGoal(EKF.state[0,0],EKF.state[1,0],1,1.5)
    A.CalcOutput()
    S.InputUpdate(A.Velocity,A.Angle)
    S.Move()
    A.StateUpdate(S.state[0,0],S.state[1,0])
    """
    AA.GradCalcGoal(PF.MX,PF.MY,1,1)
    AA.CalcOutput()
    SA.InputUpdate(AA.Velocity,AA.Angle)
    SA.Move()
    AA.StateUpdate(SA.state[0,0],SA.state[1,0])
    
    AB.GradCalcGoal(R.Odostate[0,0],R.Odostate[1,0],1,1.5)
    AB.CalcOutput()
    SB.InputUpdate(AB.Velocity,AB.Angle)
    SB.Move()
    AB.StateUpdate(SB.state[0,0],SB.state[1,0])
    """
    R.rangecalc(S.state[0,0],S.state[1,0],0)
       
    PF.PFRUN(S.state[0,0],S.state[1,0],R.range,0.5,0.5)
    EKF.RUN(S.state[0,0],S.state[1,0],R.input[2,0],1500)
    
    R.PosLog()
    S.Log()
    #SA.Log() 
    #SB.Log()  
    EKF.Log()
"""
plt.figure(1)
plt.plot(PF.particles[:,0],PF.particles[:,1],'bo',markersize = 0.5)
plt.figure(2)
plt.plot(PF.particles[:,0],PF.particles[:,2],'bo',markersize = 0.5)
plt.figure(1)
"""
R.InputUpdate(1.2,1.4,0.4,-54,0)
S.InputUpdate(1,35)
PF.Input(R.input)
EKF.InputUpdate(R.input)

"""
plt.figure(2)
plt.plot(PF.particles[:,0],PF.particles[:,2],'go',markersize = 0.5,label = 'Converged Particles')
plt.figure(1)
"""
for i in range(0,200):
    
    R.Movement()
    R.Odometry()
    
    A.GradCalcGoal(EKF.state[0,0],EKF.state[1,0],1,1.5)
    A.CalcOutput()
    S.InputUpdate(A.Velocity,A.Angle)
    S.Move()
    A.StateUpdate(S.state[0,0],S.state[1,0])
    """
    AA.GradCalcGoal(PF.MX,PF.MY,1,1)
    AA.CalcOutput()
    SA.InputUpdate(AA.Velocity,AA.Angle)
    SA.Move()
    AA.StateUpdate(SA.state[0,0],SA.state[1,0])
    
    AB.GradCalcGoal(R.Odostate[0,0],R.Odostate[1,0],1,1.5)
    AB.CalcOutput()
    SB.InputUpdate(AB.Velocity,AB.Angle)
    SB.Move()
    AB.StateUpdate(SB.state[0,0],SB.state[1,0])
    """
    R.rangecalc(S.state[0,0],S.state[1,0],0)
      
    PF.PFRUN(S.state[0,0],S.state[1,0],R.range,0.5,0.5)
    EKF.RUN(S.state[0,0],S.state[1,0],R.input[2,0],1500)
    
    R.PosLog()
    S.Log()
    #SA.Log() 
    #SB.Log()  
    EKF.Log()
"""    
plt.figure(1)
plt.plot(PF.particles[:,0],PF.particles[:,1],'bo',markersize = 0.5)
plt.figure(2)
plt.plot(PF.particles[:,0],PF.particles[:,2],'bo',markersize = 0.5)
plt.figure(1)
"""

timing = (time.time() - start_time)
print("--- %s seconds ---" % (time.time() - start_time))




"""
Error Calculation
"""
Errorx = R.state[0,0] - PF.MX
Errory = R.state[1,0] - PF.MY
Errorz = R.state[2,0] - PF.MZ
EkErrorx = R.state[1,0] - EKF.state[0,0]
EkErrory = R.state[1,0] - EKF.state[1,0]
EkErrorz = R.state[2,0] - EKF.state[2,0]
ODOErrx = R.state[0,0] - R.Odostate[0,0]
ODOErry = R.state[1,0] - R.Odostate[1,0]
ODOErrz = R.state[2,0] - R.Odostate[2,0]



PFErrx = []
PFErry = []
PFErrz = []
ODErrx = []
ODErry = []
ODErrz = []

EKFErrLogx =[]
EKFErrLogy =[]
EKFErrLogz =[]

Xaxis = []

C = 0

for i in range (0,len(PF.logx)):
    PFErrx.append(PF.logx[i] - R.GroundTruthx[i])
    PFErry.append(PF.logy[i] - R.GroundTruthy[i])
    PFErrz.append(PF.logz[i] - R.GroundTruthz[i])
    
    ODErrx.append(R.OdoLogx[i] - R.GroundTruthx[i])
    ODErry.append(R.OdoLogy[i] - R.GroundTruthy[i])
    ODErrz.append(R.OdoLogz[i] - R.GroundTruthz[i]) 
    
    EKFErrLogx.append(EKF.logx[i] - R.GroundTruthx[i])     
    EKFErrLogy.append(EKF.logy[i] - R.GroundTruthy[i]) 
    EKFErrLogz.append(EKF.logz[i] - R.GroundTruthz[i]) 
    
    Xaxis.append(C)
    C += 0.5

"""

"""
#Plotting Area
"""
plt.title('Particle Filter - Odometry - Ground Truth - APF - X/Y Plane')
plt.plot(PF.particles[:,0],PF.particles[:,1],'bo',markersize = 0.5,label = 'Final Particles')   
plt.plot(R.GroundTruthx,R.GroundTruthy,'g:',linewidth = 1, markersize = 1.5,label = 'Ground Truth')
plt.plot(S.logx,S.logy,'m--',linewidth = 1, markersize = 1,label = 'Surface Path (EKF)')
#plt.plot(SA.logx,SA.logy,'c--',linewidth = 1, markersize = 1,label = 'Surface Path (ODO)')
#plt.plot(SB.logx,SB.logy,'g--',linewidth = 1, markersize = 1,label = 'Surface Path (PF)')
plt.plot(EKF.logx,EKF.logy,'y-',linewidth = 1, markersize = 0.5,label = 'EKF')
plt.plot(R.OdoLogx,R.OdoLogy,'r-.',linewidth = 1, markersize = 1,label = 'Odometry')
plt.plot(PF.MX,PF.MY, 'k+')
plt.xlabel('East-West (Metres)')
plt.ylabel('North-South (Metres)')
plt.tight_layout()
plt.legend(loc=4)



plt.figure(2)
plt.title('Particle Filter - Odometry - Ground Truth - X/Z Plane')
plt.plot(PF.particles[:,0],PF.particles[:,2],'bo',markersize = 0.5,label = 'Final Particles')   
plt.plot(R.GroundTruthx,R.GroundTruthz,'g:',linewidth = 1, markersize = 1.5,label = 'Ground Truth')
plt.plot(R.OdoLogx,R.OdoLogz,'r-.',linewidth = 1, markersize = 1,label = 'Odometry') 
plt.plot(EKF.logx,EKF.logz,'y-',linewidth = 1, markersize = 5,label = 'EKF')
plt.plot(PF.MX,PF.MZ, 'k+')

plt.xlabel('East-West (Metres)')
plt.ylabel('Depth (Metres)')
plt.tight_layout()
plt.legend()


plt.figure(3)

plt.subplot(311)
plt.title('X Axis Error over time')
plt.plot(Xaxis,PFErrx,'b-',linewidth = 1.5, label = 'Particle Filter Error')
plt.plot(Xaxis,ODErrx,'r--',linewidth = 1.5, label = 'Odometry Error')
plt.plot(Xaxis,EKFErrLogx,'g:',linewidth = 1.5, label = 'EKF Error')
plt.xlabel('Time (Seconds)')
plt.ylabel('Error (Metres)')
plt.tight_layout()
plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)


plt.subplot(312)
plt.plot(Xaxis,PFErry,'b-',linewidth = 1.5,label = 'Particle Filter Error')
plt.plot(Xaxis,ODErry,'r--',linewidth = 1.5,label = 'Odometry Error')
plt.plot(Xaxis,EKFErrLogy,'g:',linewidth = 1.5, label = 'EKF Error')
plt.title('Y Axis Error over time')
plt.xlabel('Time (Seconds)')
plt.ylabel('Error (Metres)')
plt.tight_layout()
plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)

plt.subplot(313)
plt.plot(Xaxis,PFErrx,'b-',linewidth = 1.5,label = 'Particle Filter Error')
plt.plot(Xaxis,ODErrz,'r--',linewidth = 1.5,label = 'Odometry Error')
plt.plot(Xaxis,EKFErrLogz,'g:',linewidth = 1.5, label = 'EKF Error')
plt.title('Z Axis Error over time')
plt.xlabel('Time (Seconds)')
plt.ylabel('Error (Metres)')
plt.tight_layout()
plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)


plt.show()
"""

print(Errorx,Errory,Errorz, EkErrorx,EkErrory,EkErrorz)
print(len(PF.logx))
print(len(EKFErrLogx))

Errfile.write("{} {} {} {}  \r\n".format ("PF = ", Errorx, Errory, Errorz))
Errfile.close()

