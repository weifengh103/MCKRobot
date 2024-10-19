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
    Joints = [0]*5
    
    tmJointJoint = [np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4])]
    tmBaseJioint = [np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4])]
    
    #  
    PoseTCP = [0,0,20,0,-180,0]

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
        jointAngles = self._ks.SolveIK(self._initialTCPPose,self.PoseTCP,True)
     
        # self._ks.SolveFK(self.tmBaseJioint,self.tmJointJoint, jointAngles, self.pJoints, self.pDispTCP)

        self.tmJointJoint, self.tmBaseJioint, self.pJoints, self.pDispTCP= self._ks.SolveFK( jointAngles )
        self._ks.getTCPPoseFromTMBaseJoint(self.tmBaseJioint,self._initialTCPPose)
        # print(self.tmBaseJioint[5])
        self.UpdateRobotStatus()
       
         
    def JogRobot(self, step, poseIndex):
        self._initialTCPPose[poseIndex] = self._initialTCPPose[poseIndex] +step
        jointAngles = self._ks.SolveIK(self._initialTCPPose,self.PoseTCP,True)

        self.tmJointJoint,  self.tmBaseJioint,self.pJoints, self.pDispTCP = self._ks.SolveFK(jointAngles)
       
        self.UpdateRobotStatus()
    
    # def moveLSingle(self, pFrom, pTo):
        
    #     self._ks.UpdateIKOneToThreeJoints(pTo,True)
    #     self._ks.UpdateFK(self.tmBaseJioint,self.tmJointJoint, self.currThetas, self.pJoints, self.pDispTCP)
    #     self.UpdateRobotStatus()
        
    def JogJoint(self, index, step):
        self.currThetas[index] = self.currThetas[index] + math.radians(step)
        self._ks.UpdateFK(self.tmBaseJioint,self.tmJointJoint, self.currThetas, self.alphas, self.a, self.d, self.pJoints, self.pDispTCP)
        self.UpdateRobotStatus()

    def UpdateRobotStatus(self):
        
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

        transformation_matrix = self.tmBaseJioint[5]

        # Extract the translation vector
        x = transformation_matrix[0, 3]
        y = transformation_matrix[1, 3]
        z = transformation_matrix[2, 3]

        # Extract the rotation matrix (3x3 top-left part of the transformation matrix)
        rotation_matrix = transformation_matrix[0:3, 0:3]

        # Create a Rotation object from the rotation matrix
        rotation = Rotation.from_matrix(rotation_matrix)

        # Convert the rotation matrix to Euler angles in the XYZ order
        euler_angles = rotation.as_euler('xyz', degrees=True)  # Use degrees=False for radians

        # Extract Euler angles
        rx, ry, rz = euler_angles

        # Combine into the desired format
        self.PosRobot = (x, y, z, rx, ry, rz)

    



