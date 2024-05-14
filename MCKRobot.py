import numpy as np
import time
import matplotlib.pyplot as plt
import matplotlib as mpl
import math
from KinematicSolver import KinematicSolver as KS

class MCKRobot:
    
    
    resolutionMoveL = 0.1 #0.1 mm
    resolutionMovej = 0.1 #0.1 deg
    
    
    #DH parameters
    
    # a is distance between the origin of frame n and n-1 along Xn
    a = [0,50,0,0,0,0]
    
    # alpha is anangle from  Zn-1 to Zn along Xn
    alphas = [math.radians(90),math.radians(0),math.radians(90),
              math.radians(90),math.radians(90),math.radians(0)]
    
    # d is the distance from Xn-1 to Xn along the Zn-1 direction.
    d = [50,0,0,50,0,30]
    
    # Joint angles and positions
    J1,J2,J3,J4,J5,J6 = 0.0,0.0,0.0,0.0,0.0,0.0
    
    tmJointJoint = [np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4])]
    tmBaseJioint = [np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4])]
    
    
    Joints = [J1,J2,J3,J4,J5,J6]
    
    PoseTCP = []
    
    theta1 = math.radians(0)
    theta2 = math.radians(45)
    theta3 = math.radians(45)
    
    theta4 = math.radians(0)
    theta5 = math.radians(-90)
    theta6 = math.radians(0)
    
    currThetas = [theta1,theta2,theta3,theta4,theta5,theta6]
    
    pBase = np.array([0,0,0,1])
    pShoulder = np.array([0,0,0,1])
    pElbow = np.array([0,0,0,1])
    pElbow2 = np.array([0,0,0,1])
    pWristPos = np.array([0,0,0,1])
    pFlange = np.array([0,0,0,1])
    pTCPPos = np.array([0,0,0,1])
    
    pJoints = [pBase,pShoulder,pElbow,pElbow2,pWristPos,pFlange,pTCPPos]
    
    pDispTCPOrig = np.array([0,0,0,1])
    pDispTCPX = np.array([0,0,0,1])
    pDispTCPY = np.array([0,0,0,1])
    pDispTCPZ = np.array([0,0,0,1])
    pDispTCP = [pDispTCPOrig,pDispTCPX,pDispTCPY,pDispTCPZ]
    
    _ks = KS()

    
    def __init__(self):
        self.InitRobot()
        
        pass 
        
    
    def InitRobot(self):
        self._ks.UpdateFK(self.tmBaseJioint,self.tmJointJoint, self.currThetas, self.alphas, self.a, self.d, self.pJoints, self.pDispTCP)
        self._ks.getTCPPoseFromTMBaseJoint(self.tmBaseJioint,self.PoseTCP)
        self.mapJointsToRobotp()
         
    def JogRobot(self, step, axisIndex):
        pTo = self.PoseTCP[axisIndex] + step
        
        # Jog in World XYZ
        if(axisIndex<3):
            self.moveL(self.PoseTCP, pTo)
        else:
            pass
    
    def moveLSingle(self, pFrom, pTo):
        
        self._ks.UpdateIKOneToThreeJoints(pTo,True, self.a, self.d)
        self._ks.UpdateFK(self.tmBaseJioint,self.tmJointJoint, self.currThetas, self.alphas, self.a, self.d, self.pJoints, self.pDispTCP)
        self.mapJointsToRobotp()
        pass 
        
    

    
    # def move(self,x,y,z):
        
    #     self._ks.UpdateIKOneToThreeJoints(x,y,z,True,self.thetas, self.alphas, self.a, self.d)
    #     self._ks.UpdateFK(self.tmBaseJioint,self.tmJointJoint, self.thetas, self.alphas, self.a, self.d, self.pJoints, self.pDispTCP)
    #     self.mapJointsToRobotp()
        
    def JogJoint(self, index, step):
        self.currThetas[index] = self.currThetas[index] + math.radians(step)
        self._ks.UpdateFK(self.tmBaseJioint,self.tmJointJoint, self.currThetas, self.alphas, self.a, self.d, self.pJoints, self.pDispTCP)
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
 

    



