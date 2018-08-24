# -*- coding: utf-8 -*-
"""
Created on Fri Aug 10 20:26:46 2018

@author: Tyler
"""
import numpy as np



class PF:
    def __init__(self,Xrange,Yrange,Zrange,Anglerange,Amount,Distribution,GaussMean,GaussianVariance):
        #Initialise the particles with variables for X.Y. bearing.Weights and range readings
        self.particles = np.empty((Amount,6))
        #Uniform Distribution in an area
        if  "U" in Distribution:
            self.particles[:,0] = np.random.uniform(Xrange[0], Xrange[1], size = Amount) # X location for particles
            self.particles[:,1] = np.random.uniform(Yrange[0], Yrange[1], size = Amount) # Y location for particles
            self.particles[:,2] = np.random.uniform(Zrange[0], Zrange[1], size = Amount) # Z Location
            self.particles[:,3] = np.random.uniform(Anglerange[0], Anglerange[1], size = Amount) # bearings for particles
            self.particles[:,4] = 1/Amount # weights of particles
            self.particles[:,5] = 0 # Range measured from particle to surface bot
        #Gaussian Distribution with a set variance
        elif "G" in Distribution:
            self.particles[:,0] = np.random.normal(GaussMean[0],GaussianVariance, size = Amount) # X location for particles
            self.particles[:,1] = np.random.normal(GaussMean[1],GaussianVariance, size = Amount) # Y location for particles
            self.particles[:,2] = np.random.normal(GaussMean[2],GaussianVariance, size = Amount) # Z Location
            self.particles[:,3] = np.random.normal(GaussMean[3],GaussianVariance, size = Amount) # bearings for particles
            self.particles[:,4] = 1/Amount # weights of particles
            self.particles[:,5] = 0 # Range measured from particle to surface bot
            
        self.Amount = Amount # Total number of particles
        
        self.input = np.empty((5,1)) #matrix for input
        
        #Variabels for redistrobution
        self.GVar = GaussianVariance
        self.GMean = GaussMean
        self.Distro = Distribution
        self.Xrange = Xrange
        self.Yrange = Yrange
        self.Zrange = Zrange
        
        self.logx = []
        self.logy = []
        self.logz = []
        
        self.MX = GaussMean[0]
        self.MY = GaussMean[1]
        self.MZ = GaussMean[2]
        
        
    def Input(self,In):
        #The input values for the motion model
        self.input = In
        """
        self.input[0,0] = In[0] #Xv
        self.input[1,0] = In[1] #Yv
        self.input[2,0] = In[2] #Zv
        self.input[3,0] = np.radians(In[3]) #Bearing
        self.input[4,0] = np.radians(In[4]) #Pitch
        """
    
    def movement(self,dt):
        #particles are moved through the same motion model as the robot
        for i in range (0,(self.Amount)):
            self.particles[i,0] = self.particles[i,0] + dt*((self.input[0,0]*np.cos(self.input[3,0]))-(self.input[1,0]*np.sin(self.input[3,0]))) #+ np.random.normal(0,4)
            self.particles[i,1] = self.particles[i,1] + dt*((self.input[0,0]*np.sin(self.input[3,0]))+(self.input[1,0]*np.cos(self.input[3,0]))) #+ np.random.normal(0,4)
            self.particles[i,2] = self.particles[i,2] + dt* ((self.input[0,0]*np.sin(self.input[4,0]))+(self.input[2,0]*np.cos(self.input[4,0])))# + np.random.normal(0,4)
   
    def rangecalc(self,X2,Y2):
        #Measures the distance from the surface robot to the particle
        for i in range (0,(self.Amount)):
            self.particles[i,5] = np.sqrt((self.particles[i,0] - X2)**2 + (self.particles[i,1] - Y2)**2 + (self.particles[i,2])**2) + np.random.normal(0,2)
     
    def Weighting(self,RobotRange):
        #updates the weights of the particles based on their comparison to the robot range
        for i in range (0,(self.Amount)):
            self.particles[i,4] = RobotRange/self.particles[i,5]
            if self.particles[i,4] >1:
                self.particles[i,4] = self.particles[i,5]/RobotRange
            
        
        self.maximum = np.amax(self.particles[:,4])
        self.minimum = np.amin(self.particles[:,4])
        self.mean = np.mean(self.particles[:,4])
        self.median = np.median(self.particles[:,4])
        
    def convergence(self):
        #randomly selects from the more likely particles to resample
        indexL = []
        indexM = []
        for i in range (0,self.Amount):
            if self.particles[i,4] <= self.mean**2:
                indexL.append(i)
            if self.particles[i,4] > self.mean:
                indexM.append(i)

        length = len(indexL)

        for i in range (0,length-1):
            N = np.random.choice(indexM)
            self.particles[indexL[i],0] = self.particles[N,0]
            self.particles[indexL[i],1] = self.particles[N,1]
            self.particles[indexL[i],2] = self.particles[N,2]
        
        Total = sum(self.particles[:,4])
        for i in range (0,(self.Amount)):
            self.particles[i,4] = self.particles[i,4]/Total
            
            
    def WeightMean(self):
        Sx = 0 #Weighted Sum of X positions times weights
        Sy = 0 #Weighted sum of Y positions times weights
        Sz = 0 #Weighted Sum of Z poition times weights
        WT = 0 # Weight total
        self.MX = 0
        self.MY = 0
        self.MZ = 0
        for i in range (0, (self.Amount)):
            Sx += self.particles[i,0] * self.particles[i,4]
            Sy += self.particles[i,1] * self.particles[i,4]
            Sz += self.particles[i,2] * self.particles[i,4]
            WT += self.particles[i,4]
    
        self.MX = Sx/WT
        self.MY = Sy/WT
        self.MZ = Sz/WT
        
    def Redistibute(self,Threshold):
        DiffX = np.amax(self.particles[:,0]) - np.amin(self.particles[:,0])
        DiffY = np.amax(self.particles[:,1]) - np.amin(self.particles[:,1])
        DiffZ = np.amax(self.particles[:,2]) - np.amin(self.particles[:,2])
        
        if DiffX < Threshold or DiffY < Threshold or DiffZ < Threshold:
            if  "U" in self.Distro:
                for i in range(0,self.Amount):
                    self.particles[i,0] = np.random.uniform((self.MX-self.Xrange[0]),self.Xrange[1]) # X location for particles
                    self.particles[i,1] = np.random.uniform((self.MY-self.Yrange[0]), self.Yrange[1]) # Y location for particles
                    self.particles[i,2] = np.random.uniform((self.MZ-self.Zrange[0]), self.Zrange[1]) # Z Location
                    self.particles[i,4] = 1/self.Amount # weights of particles
                    self.particles[i,5] = 0 # Range measured from particle to surface bot
            #Gaussian Distribution with a set variance
            elif "G" in self.Distro:
                for i in range(0,self.Amount):
                    self.particles[i,0] = np.random.normal(self.MX,self.GVar) # X location for particles
                    self.particles[i,1] = np.random.normal(self.MY,self.GVar) # Y location for particles
                    self.particles[i,2] = np.random.normal(self.MZ,self.GVar) # Z Location
                    self.particles[i,4] = 1/self.Amount # weights of particles
                    self.particles[i,5] = 0 # Range measured from particle to surface bot
            print("Particles Redistributed")
    def Log(self):
        self.logx.append(self.MX)
        self.logy.append(self.MY)
        self.logz.append(self.MZ)
        
        
    def PFRUN(self,X2,Y2,RobotRange,dt,Threshold):
        self.movement(dt)
        self.rangecalc(X2,Y2)
        self.Weighting(RobotRange)
        self.convergence()
        self.WeightMean()
        self.Redistibute(Threshold)
        self.Log()

