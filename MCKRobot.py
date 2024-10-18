import numpy as np
import math
from KinematicSolver import KinematicSolver as KS
from DHMatrix6Dof import DHMatrix6Dof as DHPara
class MCKRobot:
    
    #Load DH parameters
    _a = DHPara.A
    _alphas = DHPara.Alphas
    _d = DHPara.D
    _thetas = DHPara.Thetas
    
    # Joint angles and positions
    # J1,J2,J3,J4,J5,J6 = 0.0,0.0,0.0,0.0,0.0,0.0
    Joints = [0]*5
    
    _tmJointJoint = [np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4])]
    _tmBaseJioint = [np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4])]
    
    inputAngleDeg = [0,0,0,0,0,0]
    
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
    
    _ks = None

    # change this for setting initial robot position
    _initialTCPPose = [50,0,50,180,0,0]

    def __init__(self):
        self._ks = KS(self._a,self._d,self._alphas,self._thetas)
        jointAngles = self._ks.SolveIK(self._initialTCPPose,True)
     
        # self._ks.SolveFK(self._tmBaseJioint,self._tmJointJoint, jointAngles, self.pJoints, self.pDispTCP)

        self._tmJointJoint, self._tmBaseJioint, self.pJoints, self.pDispTCP= self._ks.SolveFK( jointAngles )
        self._ks.getTCPPoseFromTMBaseJoint(self._tmBaseJioint,self._initialTCPPose)
        # print(self._tmBaseJioint[5])
        self.mapJointsToRobotp()
       
         
    def JogRobot(self, step, poseIndex):
        self._initialTCPPose[poseIndex] = self._initialTCPPose[poseIndex] +step
        jointAngles = self._ks.SolveIK(self._initialTCPPose,True)
        self.inputAngleDeg = jointAngles

        self._tmJointJoint,  self._tmBaseJioint,self.pJoints, self.pDispTCP = self._ks.SolveFK( self.inputAngleDeg)
       
        self.mapJointsToRobotp()
    
    def moveLSingle(self, pFrom, pTo):
        
        self._ks.UpdateIKOneToThreeJoints(pTo,True)
        self._ks.UpdateFK(self._tmBaseJioint,self._tmJointJoint, self.currThetas, self.pJoints, self.pDispTCP)
        self.mapJointsToRobotp()
        
    def JogJoint(self, index, step):
        self.currThetas[index] = self.currThetas[index] + math.radians(step)
        self._ks.UpdateFK(self._tmBaseJioint,self._tmJointJoint, self.currThetas, self.alphas, self.a, self.d, self.pJoints, self.pDispTCP)
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
 

    



