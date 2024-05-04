import numpy as np
import time
import matplotlib.pyplot as plt
import matplotlib as mpl
import math
from KinematicSolver import KinematicSolver as KS

class MCKRobot:
    
 
    #DH parameters
    #weifeng 
    a = [0,50,50,0,0,0]
    alphas = [math.radians(90),math.radians(0),math.radians(0),math.radians(0),math.radians(90),math.radians(0)]
    d = [50,0,0,0,0,0]
    
    # Joint angles and positions
    J1,J2,J3,J4,J5,J6 = 0.0,0.0,0.0,0.0,0.0,0.0
    
    
    Joints = [J1,J2,J3,J4,J5,J6]
    
    theta1 = math.radians(0)
    theta2 = math.radians(0)
    theta3 = math.radians(0)
    theta4 = math.radians(0)
    theta5 = math.radians(0)
    theta6 = math.radians(-45)
    thetas = [theta1,theta2,theta3,theta4,theta5,theta6]
    
    pBase = np.array([0,0,0,1])
    pShoulder = np.array([0,0,0,1])
    pElbow = np.array([0,0,0,1])
    # pWrist = np.array([0,0,0,1])
    
    pWristPos = np.array([0,0,0,0,0,0])
    pFlangePos = np.array([0,0,0,0,0,0])
    pTCPPos = np.array([0,0,0,0,0,0])
    
    pJoints = [pBase,pShoulder,pElbow,pWristPos,pTCPPos]
    
    # Links
    # LinkBaseSholder = [[pBase[0],pBase[1],pBase[2]],[pShoulder[0],pShoulder[1],pShoulder[2]]]
    # LinkSholderElbow = [[pShoulder[0],pShoulder[1],pShoulder[2]],[pElbow[0],pElbow[1],pElbow[2]]]
    # LinkElbowWrist = [[pElbow[0],pElbow[1],pElbow[2]],[pWristPos[0],pWristPos[1],pWristPos[2]]]
    _ks = KS()

    
    def __init__(self):
        pass 
        
    
    def InitRobot(self):
        self._ks.UpdateFK(self.thetas, self.alphas, self.a, self.d, self.pJoints)
        self.mapJointsToRobotp()
    
    def move(self,x,y,z):
        self._ks.UpdateIKOneToThreeJoints(x,y,z,True,self.thetas, self.alphas, self.a, self.d)
        self._ks.UpdateFK(self.thetas, self.alphas, self.a, self.d, self.pJoints)
        self.mapJointsToRobotp()
        
    def mapJointsToRobotp(self):
        
        self.pBase = self.pJoints[0]
        self.pShoulder =self.pJoints[1]
        self.pElbow = self.pJoints[2]
        self.pWristPos = self.pJoints[3]
        self.pTCPPos = self.pJoints[4]
        
        
        # self.pTCP = self.pJoints[4]

    



