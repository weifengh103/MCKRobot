import numpy as np
import math
from KinematicSolver import KinematicSolver as KS
from DHMatrix6Dof import DHMatrix6Dof as DHPara
from scipy.spatial.transform import Rotation   

class MCKRobot:
    
    #Load DH parameters
    _a = DHPara.A
    _alphas = DHPara.Alphas
    _d = DHPara.D
    _thetas = DHPara.Thetas
    
    # Joint angles and positions
    # J1,J2,J3,J4,J5,J6 = 0.0,0.0,0.0,0.0,0.0,0.0
    Joints = [0]*6
    
    tmJointJoint = [np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4])]
    tmBaseJioint = [np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4])]
    tmBaseTCP = [np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4])]
    tmBaseFlange = [np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4])]
    #  
    TCP = [0,0,0,0,0,0]

    pBase = np.array([0,0,0,1])
    pShoulder = np.array([0,0,0,1])
    pElbow = np.array([0,0,0,1])
    pElbow2 = np.array([0,0,0,1])
    pWristPos = np.array([0,0,0,1])
    pFlange = np.array([0,0,0,1])
    pTCPPos = np.array([0,0,0,1])
    pJoints = [pBase,pShoulder,pElbow,pElbow2,pWristPos,pFlange,pTCPPos]
    
    #pDispTCP with orig, x,y,z points
    pDispTCP = [[0,0,0,1],[0,0,0,1],[0,0,0,1],[0,0,0,1]]
    
    _ks = None

    # change this for setting initial robot position
    _initialTCPPose = [50,0,50,180,0,0]

    def __init__(self):
        
        self._ks = KS(self._a,self._d,self._alphas,self._thetas)
        jointAngles = self._ks.SolveIK(self._initialTCPPose,self.TCP,True)
        self.tmJointJoint, self.tmBaseJioint, self.tmBaseFlange, self.tmBaseTCP= self._ks.SolveFK( jointAngles,self.TCP)
        # self._ks.getTCPPoseFromTMBaseJoint(self.tmBaseJioint,self._initialTCPPose)
        self.UpdateRobotStatusV2( self.tmBaseJioint, self.tmBaseTCP)
         
    def JogRobot(self, step, poseIndex):

        # self._initialTCPPose[poseIndex] = self._initialTCPPose[poseIndex] +step
        # self._initialTCPPose[3] = 45
        jointAngles = self._ks.SolveIK(self._initialTCPPose,self.TCP,True)
        self.tmJointJoint,self.tmBaseJioint,self.tmBaseFlange, self.tmBaseTCP = self._ks.SolveFK(jointAngles,self.TCP)
        self.UpdateRobotStatusV2( self.tmBaseJioint, self.tmBaseTCP)
        

    # def JogRobot(self, step, poseIndex):

    #     self._initialTCPPose[poseIndex] = self._initialTCPPose[poseIndex] +step
    #     jointAngles,tmBaseToTCP = self._ks.SolveIK(self._initialTCPPose,self.TCP,True)
    #     self.tmJointJoint,  self.tmBaseJioint,self.tmBaseTCP, self.pJoints, self.pDispTCP = self._ks.SolveFK(jointAngles,tmBaseToTCP)
    #     self.UpdateRobotStatus()
    
    def JogRobotTCP(self, step, poseIndex):

        TCPLocalOffset = np.zeros(6)
        TCPLocalOffset[poseIndex] = step
        tmTCPLocalOffset = self._ks.PoseToTransformationMatrix(TCPLocalOffset)

        tmIP = self._ks.PoseToTransformationMatrix(self._initialTCPPose)

        targetPose = self._ks.TransformationMatrixToPose(np.matmul(tmIP,tmTCPLocalOffset))

        jointAngles = self._ks.SolveIK(targetPose,self.TCP,True)
        self.tmJointJoint,  self.tmBaseJioint,self.pJoints= self._ks.SolveFK(jointAngles)


        self.UpdateRobotStatus()
    
    def JogJoint(self, index, step):

        # self.currThetas[index] = self.currThetas[index] + math.radians(step)
        self.tmJointJoint,self.tmBaseJioint,self.tmBaseFlange, self.tmBaseTCP = self._ks.SolveFK(self.Joints,self.TCP)

        self.UpdateRobotStatusV2( self.tmBaseJioint, self.tmBaseTCP)

    # def UpdateRobotStatus(self):
        
    


    #     self.pBase = self.pJoints[0]
    #     self.pShoulder =self.pJoints[1]
    #     self.pElbow = self.pJoints[2]
    #     self.pElbow2 = self.pJoints[3]
    #     self.pWristPos = self.pJoints[4]
    #     self.pFlange = self.pJoints[5]
    #     self.pTCPPos = self.pJoints[6]
        
    #     transformation_matrix = self.tmBaseJioint[5]

    #     x = transformation_matrix[0, 3]
    #     y = transformation_matrix[1, 3]
    #     z = transformation_matrix[2, 3]
    #     rotation_matrix = transformation_matrix[0:3, 0:3]
    #     rotation = Rotation.from_matrix(rotation_matrix)

    #     euler_angles = rotation.as_euler('xyz', degrees=True)  
    #     rx, ry, rz = euler_angles

    #     # Combine into the desired format
    #     self.PosRobot = (x, y, z, rx, ry, rz)

    def UpdateRobotStatusV2(self,tmBaseJioint, tmBaseTCP):
        
        self.pJoints[1] = np.matmul(tmBaseJioint[0] , self.pJoints[0]).A1
        self.pJoints[2] = np.matmul(tmBaseJioint[1] , self.pJoints[0]).A1
        self.pJoints[3] = np.matmul(tmBaseJioint[2] , self.pJoints[0]).A1
        self.pJoints[4] = np.matmul(tmBaseJioint[3] , self.pJoints[0]).A1
        self.pJoints[5] = np.matmul(tmBaseJioint[4] , self.pJoints[0]).A1
        self.pJoints[6] = np.matmul(tmBaseJioint[5] , self.pJoints[0]).A1

        self.pBase = self.pJoints[0]
        self.pShoulder =self.pJoints[1]
        self.pElbow = self.pJoints[2]
        self.pElbow2 = self.pJoints[3]
        self.pWristPos = self.pJoints[4]
        self.pFlange = self.pJoints[5]
        self.pTCPPos = self.pJoints[6]

        dispTCPAxisLength = 20
        self.pDispTCP[0] = np.matmul(tmBaseTCP , [0,0,0,1]).A1
        self.pDispTCP[1] = np.matmul(tmBaseTCP , [dispTCPAxisLength,0,0,1]).A1
        self.pDispTCP[2] = np.matmul(tmBaseTCP , [0,dispTCPAxisLength,0,1]).A1
        self.pDispTCP[3] = np.matmul(tmBaseTCP , [0,0,dispTCPAxisLength,1]).A1

        self.PosRobot = (self._ks.TransformationMatrixToPose(tmBaseTCP))
        

    



