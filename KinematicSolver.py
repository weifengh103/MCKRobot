import numpy as np
import time
import matplotlib.pyplot as plt
import matplotlib as mpl
import math
from scipy.spatial.transform import Rotation   

class KinematicSolver:
    
    a = [0] * 6
    d = [0] * 6
    alphas = [0] *6
    
    def __init__(self,a,d,alphas,thetas):
        self.a = a
        self.d = d
        self.alphas = alphas
        self.thetas = thetas
        pass
     
    # FK Section
    
    def getDHTransMatrix(self,i,angelDeg):
        newTheta = self.thetas[i] + math.radians( angelDeg[i])

        # See how nicely they are aligned :D
        dhMatrix = np.matrix([
            [math.cos(newTheta), -math.sin(newTheta) * math.cos(self.alphas[i]),  math.sin(newTheta) * math.sin(self.alphas[i]), self.a[i] * math.cos(newTheta)],
            [math.sin(newTheta),  math.cos(newTheta) * math.cos(self.alphas[i]), -math.cos(newTheta) * math.sin(self.alphas[i]), self.a[i] * math.sin(newTheta)],
            [0,                   math.sin(self.alphas[i]),                       math.cos(self.alphas[i]),                      self.d[i]],
            [0,                   0,                                              0,                                             1]
            ])
        
        return dhMatrix

    def updateAllTJointJointTrans(self,angelDeg):
        return [self.getDHTransMatrix(i,angelDeg) for i in range(6)]
    
    def updateAllTBaseJointTrans(self,tmJointJoint):
         # Update first four base to joint transition matrix based on DH Matrix
        tmBaseJioint = [None]*6
        for i in range(6):
            if i == 0:
                tmBaseJioint[i] = tmJointJoint[i]
            else:
                tmBaseJioint[i] = np.matmul(tmBaseJioint[i-1] , tmJointJoint[i])
        return tmBaseJioint
                


    def PoseToTransformationMatrix(self,pose,isExtrinsic):
        # Convert Euler angles (rx, ry, rz) from degrees to a rotation matrix
        v = pose[-3:]
        if isExtrinsic:
            rm = Rotation.from_euler('xyz', pose[-3:], degrees=True).as_matrix()
        else: 
            rm = Rotation.from_euler('XYZ', pose[-3:], degrees=True).as_matrix()
        tm = np.eye(4)   
        tm[0:3, 0:3] = rm   
        tm[0:3, 3] = [pose[0],pose[1],pose[2]]  

        return tm
    
    def TransformationMatrixToPose(self,tm,isExtrinsic):
        x = tm[0, 3]
        y = tm[1, 3]
        z = tm[2, 3]

        # Extract the rotation matrix (3x3 top-left part of the transformation matrix)
        rotation_matrix = tm[0:3, 0:3]

        # Create a Rotation object from the rotation matrix
        if isExtrinsic:
            euler_angles = Rotation.from_matrix(rotation_matrix).as_euler('xyz', degrees=True)  
        else:
            euler_angles = Rotation.from_matrix(rotation_matrix).as_euler('XYZ', degrees=True)  

        # Convert the rotation matrix to Euler angles in the xyz order
        # euler_angles = rotation.as_euler('xyz', degrees=True)  # Use degrees=False for radians

        # Extract Euler angles
        rx, ry, rz = euler_angles

        return [x,y,z,rx,ry,rz]
    
  
    def GetFlangeBase(self,tmBaseTCP,TCP):
        # pose = [50,0,50,0,0,45]

        tmTCP = self.PoseToTransformationMatrix(TCP,True)

        tmTCPInv = np.linalg.inv(tmTCP)
      

        #below        np.matmul(tmBaseTCP,tmTCPInv) give CORRCETY xyz rotation when RX RY RZ are zero
        tmBaseFlange = np.matmul(tmBaseTCP,tmTCPInv)
        
        baseFlage = self.TransformationMatrixToPose(tmBaseFlange, True)
        return baseFlage

    def SolveIK(self,TCPPose, elbowUp):

     
   
        BaseFlangePose = TCPPose

        tmJointJoint = [np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4])]
        tmBaseJioint = [np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4])]
        
        angelsRad = [0,0,0,0,0,0]
        px = BaseFlangePose[0]
        py = BaseFlangePose[1]
        pz = BaseFlangePose[2]

        #project the next new x to current x' axis and b j1
        
        angelsRad[0] = math.atan2(py,px)
        
        #  rotatedX is the same as the length of the arm on X axis 
        pxRotated = math.pow((py**2 + px**2),0.5) 
        
        if(abs(angelsRad[0])>math.pi/2):
            pxRotated = -pxRotated
       

        pzLocal = pz - self.d[0]
            
        
        # j2 and j3 calculation is absed on law of cosines and this geometric method https://www.roboticworx.io/p/how-to-calculate-inverse-kinematics
        
        # get j3
   
        j3Abs =  math.pi - math.acos((self.a[1]**2 + self.d[3]**2 - pxRotated**2 - pzLocal**2)/(2*self.a[1]*self.d[3])) 
        
        # get j2
        if(elbowUp == True):
            angelsRad[2] = - j3Abs
            angelsRad[1] = math.atan(pzLocal/pxRotated) +  math.atan(self.d[3]*math.sin(j3Abs)/(self.a[1] + self.d[3]*math.cos(j3Abs)))
            
        else:
            angelsRad[2] = j3Abs
            angelsRad[1] = math.atan(pzLocal/pxRotated) -  math.atan(self.d[3]*math.sin(j3Abs)/(self.a[1] + self.d[3]*math.cos(j3Abs)))
        
        # process for getting j4, j5 and j6 
                
        rX = math.radians(BaseFlangePose[3])
        rY = math.radians(BaseFlangePose[4])
        rZ = math.radians(BaseFlangePose[5])
        
        # get world  (extrinsic) rotations euler angle of flage
        rmBaseToFlange = Rotation.from_euler('xyz', [rX, rY, rZ], degrees=False).as_matrix()
        angelsDeg = np.degrees(angelsRad)
        angelsDeg = np.degrees(angelsRad)

        tmJointJoint = self.updateAllTJointJointTrans(angelsDeg)
        tmBaseJioint = self.updateAllTBaseJointTrans(tmJointJoint)

        rmBaseToLink3End = tmBaseJioint[5][:3, :3]
        rmLink3ToBaseEnd = np.linalg.inv(rmBaseToLink3End) 
        
        rbLink3EndToFlange = np.matmul(rmLink3ToBaseEnd, rmBaseToFlange)
        
        r = Rotation.from_matrix(rbLink3EndToFlange)

         # get body centred (intrinsic) rotations euler angle for j4, j5, j6
        angles =  r.as_euler('XYZ',degrees = False) 
        
        #RZ J4
        angelsRad[3] = angles[0] 
        #RY J5
        angelsRad[4] = angles[1] 
        #RX J6
        angelsRad[5] = angles[2]
        print(angles[-3:])
        return np.degrees(angelsRad)
    
    def SolveFK(self,currAngleDeg,TCP):
        tmJointJoint = self.updateAllTJointJointTrans(currAngleDeg)
        tmBaseJioint = self.updateAllTBaseJointTrans(tmJointJoint)
        tmBaseFlange = tmBaseJioint[5]
        tmBaseTCP = np.matmul(tmBaseFlange,self.PoseToTransformationMatrix(TCP,True))

        return tmJointJoint, tmBaseJioint, tmBaseFlange, tmBaseTCP
