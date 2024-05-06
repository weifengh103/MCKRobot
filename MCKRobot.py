import numpy as np
import time
import matplotlib.pyplot as plt
import matplotlib as mpl
import math
from KinematicSolver import KinematicSolver as KS

class MCKRobot:
    
 
    #DH parameters
    #weifeng 
    
    # a is distance between the origin of frame n and n-1 along Xn
    # a = [0,50,50,0,0,0]
    # a = [0,50,50,30]
    # a = [0,50,50,0,30]
    # a = [0,50,50,0,0]
    
    # new order
    # a = [0,50,0,0,30]
    # new order2
    a = [0,50,0,0,0,0]
    
    
    #Zn-1 to Zn along Xn
    # alphas = [math.radians(90),math.radians(0),math.radians(0),
    #           math.radians(0),math.radians(90),math.radians(0)]
     # new order
    # alphas = [math.radians(90),math.radians(0),math.radians(90),
    #           math.radians(90),math.radians(90),math.radians(-90)]
    # new order2
    alphas = [math.radians(90),math.radians(0),math.radians(90),
              math.radians(90),math.radians(90),math.radians(0)]
    
    
    # d is the distance from Xn-1 to Xn along the Zn-1 direction.
    # d = [50,0,0,0,0,30]
    # d = [50,0,0,0]
    # d = [50,0,0,0,0]
    # d = [50,0,0,0,30]
    d = [50,0,0,50,0,30]
    
    # Joint angles and positions
    J1,J2,J3,J4,J5,J6 = 0.0,0.0,0.0,0.0,0.0,0.0
    
    
    Joints = [J1,J2,J3,J4,J5,J6]
    
    theta1 = math.radians(0)
    theta2 = math.radians(45)
    theta3 = math.radians(45)
    
    theta4 = math.radians(0)
    theta5 = math.radians(-90)
    theta6 = math.radians(0)
    thetas = [theta1,theta2,theta3,theta4,theta5,theta6]
    
    pBase = np.array([0,0,0,1])
    pShoulder = np.array([0,0,0,1])
    pElbow = np.array([0,0,0,1])
    # pWrist = np.array([0,0,0,1])
    pElbow2 = np.array([0,0,0,1])
    pWristPos = np.array([0,0,0,1])
    pWristPos = np.array([0,0,0,1])
    pFlange = np.array([0,0,0,1])
    pTCPPos = np.array([0,0,0,1])
    
    pJoints = [pBase,pShoulder,pElbow,pElbow2,pWristPos,pFlange,pTCPPos]
    
    
    pDispTCPOrig = np.array([0,0,0,1])
    pDispTCPX = np.array([0,0,0,1])
    pDispTCPY = np.array([0,0,0,1])
    pDispTCPZ = np.array([0,0,0,1])
    
    pDispTCP = [pDispTCPOrig,pDispTCPX,pDispTCPY,pDispTCPZ]
    # Links
    # LinkBaseSholder = [[pBase[0],pBase[1],pBase[2]],[pShoulder[0],pShoulder[1],pShoulder[2]]]
    # LinkSholderElbow = [[pShoulder[0],pShoulder[1],pShoulder[2]],[pElbow[0],pElbow[1],pElbow[2]]]
    # LinkElbowWrist = [[pElbow[0],pElbow[1],pElbow[2]],[pWristPos[0],pWristPos[1],pWristPos[2]]]
    _ks = KS()

    
    def __init__(self):
        pass 
        
    
    def InitRobot(self):
        self._ks.UpdateFK(self.thetas, self.alphas, self.a, self.d, self.pJoints, self.pDispTCP)
        self.mapJointsToRobotp()
         
    
    def move(self,x,y,z):
        pass
        self._ks.UpdateIKOneToThreeJoints(x,y,z,True,self.thetas, self.alphas, self.a, self.d)
        self._ks.UpdateFK(self.thetas, self.alphas, self.a, self.d, self.pJoints,self.pDispTCP)
        self.mapJointsToRobotp()
        
    def mapJointsToRobotp(self):
        
        self.pBase = self.pJoints[0]
        self.pShoulder =self.pJoints[1]
        self.pElbow = self.pJoints[2]
        self.pElbow2 = self.pJoints[3]
        self.pWristPos = self.pJoints[4]
        self.pFlange = self.pJoints[5]
        self.pTCPPos = self.pJoints[6]
        
        self.pDispTCPOrig= self.pDispTCP[0]
        self.pDispTCPX= self.pDispTCP[1]
        self.pDispTCPY= self.pDispTCP[2]
        self.pDispTCPZ= self.pDispTCP[3]
        # self.pTCP = self.pJoints[4]

    



