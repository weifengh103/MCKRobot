import numpy as np
import time
import matplotlib.pyplot as plt
import matplotlib as mpl
import math
from KinematicSolver import KinematicSolver as KS

class MCKRobot:
    
 
    #DH parameters
    #weifeng 
    a = [0,50,50,0,0]
    alphas = [math.radians(90),math.radians(0),math.radians(0),math.radians(0),math.radians(90)]
    d = [50,0,0,0,0]
    
    # Joint angles and positions
    J1,J2,J3,J4,J5,J6 = 0.0,0.0,0.0,0.0,0.0,0.0
    
    
    Joints = [J1,J2,J3,J4,J5,J6]
    
    theta1 = math.radians(0)
    theta2 = math.radians(0)
    theta3 = math.radians(0)
    theta4 = math.radians(0)
    theta5 = math.radians(0)
    theta6 = math.radians(0)
    thetas = [theta1,theta2,theta3,theta4,theta5,theta6]
    
    pBase = np.array([0,0,0,1])
    pShoulder = np.array([0,0,0,1])
    pElbow = np.array([0,0,0,1])
    pWrist = np.array([0,0,0,1])
    pTCP = np.array([0,0,0,1])
    # Links
    LinkBaseSholder = [[pBase[0],pBase[1],pBase[2]],[pShoulder[0],pShoulder[1],pShoulder[2]]]
    LinkSholderElbow = [[pShoulder[0],pShoulder[1],pShoulder[2]],[pElbow[0],pElbow[1],pElbow[2]]]
    LinkElbowWrist = [[pElbow[0],pElbow[1],pElbow[2]],[pWrist[0],pWrist[1],pWrist[2]]]
    
    _ks = KS
    
    def __init__(self, name):
        self.name = name
        self._ks.UpdateFK(self.thetas, self.alphas, self.a, self.d, self.pBase,self.pShoulder,self.pElbow,self.pWrist,self.pTCP)
 
        
    def move(self,x,y,z):
        self._ks.UpdateIKOneToThreeJoints(x,y,z,True,self.thetas, self.alphas, self.a, self.d)
        self._ks.UpdateFK(self.thetas, self.alphas, self.a, self.d, self.pBase,self.pShoulder,self.pElbow,self.pWrist,self.pTCP)

    



