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
    
    def getDHTransMatrix(self,i,inputAngleDeg):
        newTheta = self.thetas[i] + math.radians( inputAngleDeg[i])

        # See how nicely they are aligned :D
        dhMatrix = np.matrix([
            [math.cos(newTheta), -math.sin(newTheta) * math.cos(self.alphas[i]),  math.sin(newTheta) * math.sin(self.alphas[i]), self.a[i] * math.cos(newTheta)],
            [math.sin(newTheta),  math.cos(newTheta) * math.cos(self.alphas[i]), -math.cos(newTheta) * math.sin(self.alphas[i]), self.a[i] * math.sin(newTheta)],
            [0,                   math.sin(self.alphas[i]),                       math.cos(self.alphas[i]),                      self.d[i]],
            [0,                   0,                                              0,                                             1]
            ])
        
        return dhMatrix

    def updateAllTJointJointTrans(self,inputAngleDeg):
        return [self.getDHTransMatrix(i,inputAngleDeg) for i in range(6)]
    
    def updateAllTBaseJointTrans(self,tmJointJoint):
         # Update first four base to joint transition matrix based on DH Matrix
        tmBaseJioint = [None]*6
        for i in range(6):
            if i == 0:
                tmBaseJioint[i] = tmJointJoint[i]
            else:
                tmBaseJioint[i] = np.matmul(tmBaseJioint[i-1] , tmJointJoint[i])
        return tmBaseJioint
                
    def SolveFK(self,currAngleDeg):
        pJoints = [np.array([0, 0, 0, 1]) for _ in range(7)]
        pDispTCP = [np.array([0, 0, 0, 1]) for _ in range(4)]

        tmJointJoint = self.updateAllTJointJointTrans(currAngleDeg)
        tmBaseJioint = self.updateAllTBaseJointTrans(tmJointJoint)

        pJoints[1] = np.matmul(tmBaseJioint[0] , pJoints[0]).A1
        pJoints[2] = np.matmul(tmBaseJioint[1] , pJoints[0]).A1
        pJoints[3] = np.matmul(tmBaseJioint[2] , pJoints[0]).A1
        pJoints[4] = np.matmul(tmBaseJioint[3] , pJoints[0]).A1
        pJoints[5] = np.matmul(tmBaseJioint[4] , pJoints[0]).A1
        pJoints[6] = np.matmul(tmBaseJioint[5] , pJoints[0]).A1

        dispTCPAxisLength = 20
        dispTMIndex = 5

        pDispTCP[0] = np.matmul(tmBaseJioint[dispTMIndex] , [0,0,0,1]).A1
        pDispTCP[1] = np.matmul(tmBaseJioint[dispTMIndex] , [dispTCPAxisLength,0,0,1]).A1
        pDispTCP[2] = np.matmul(tmBaseJioint[dispTMIndex] , [0,dispTCPAxisLength,0,1]).A1
        pDispTCP[3] = np.matmul(tmBaseJioint[dispTMIndex] , [0,0,dispTCPAxisLength,1]).A1
        
        return tmJointJoint, tmBaseJioint, pJoints,pDispTCP
        
      
 
    def getTCPPoseFromTMBaseJoint(self,tmBaseJoint,curTCPPose):
        
        # # Treat joint 4 -6 as a sphere joint with  3 rotational degree. Calculate base on rotation sequence x'→y'→z' 
        # # Reference: https://www.mecademic.com/academic_articles/space-orientation-euler-angles/
    
        translationTCP = tmBaseJoint[5][:3, 3].A1

        rmTCP =  tmBaseJoint[5] [:3, :3]
        r =  Rotation.from_matrix(rmTCP)
        eulerAngles = r.as_euler("xyz",degrees=True)

        #### Modify the angles
        # curTCPPose = (translationTCP.tolist() + eulerAngles.tolist())
        curTCPPose[:3] = translationTCP.tolist()
        curTCPPose[3:] = eulerAngles.tolist()
        print(curTCPPose)

    def SolveIK(self,pose,elbowUp):
        
        tmJointJoint = [np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4])]
        tmBaseJioint = [np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4])]
        
        thetas = [0,0,0,0,0,0]
        px = pose[0]
        py = pose[1]
        pz = pose[2]

        #project the next new x to current x' axis and get j1
        
        thetas[0] = math.atan2(py,px)
        
        #  rotatedX is the same as the length of the arm on X axis 
        pxRotated = math.pow((py**2 + px**2),0.5) 
        
        if(abs(thetas[0])>math.pi/2):
            pxRotated = -pxRotated
       

        pzLocal = pz - self.d[0]
            
        
        # j2 and j3 calculation is absed on law of cosines and this geometric method https://www.roboticworx.io/p/how-to-calculate-inverse-kinematics
        
        # get j3
        temp = 2*self.a[1]*self.d[3]
        temp2 = self.a[1]**2 + self.d[3]**2 - pxRotated**2 - pzLocal**2
        temp3 = temp2/temp
        j3Abs =  math.pi - math.acos((self.a[1]**2 + self.d[3]**2 - pxRotated**2 - pzLocal**2)/(2*self.a[1]*self.d[3])) 
        
        j3AbsDeg = math.degrees(j3Abs)
        
        # get j2
        if(elbowUp == True):
            thetas[2] = - j3Abs
            thetas[1] = math.atan(pzLocal/pxRotated) +  math.atan(self.d[3]*math.sin(j3Abs)/(self.a[1] + self.d[3]*math.cos(j3Abs)))
            
        else:
            thetas[2] = j3Abs
            thetas[1] = math.atan(pzLocal/pxRotated) -  math.atan(self.d[3]*math.sin(j3Abs)/(self.a[1] + self.d[3]*math.cos(j3Abs)))
        
        
        # process for getting j4, j5 and j6 
                
        rX = math.radians(pose[3])
        rY = math.radians(pose[4])
        rZ = math.radians(pose[5])
        
        # get world rotation of flage
        rmBaseToFlange = Rotation.from_euler('XYZ', [rX, rY, rZ], degrees=False).as_matrix()
    
        
        # # update first FK for geting TM base to link3 end 
        degreethetas = np.degrees(thetas)

        tmJointJoint = self.updateAllTJointJointTrans(degreethetas)
        tmBaseJioint = self.updateAllTBaseJointTrans(tmJointJoint)
        
        rmBaseToLink3End = tmBaseJioint[5][:3, :3]
        rmLink3ToBaseEnd = np.linalg.inv(rmBaseToLink3End) 

        rbLink3EndToFlange = np.matmul(rmLink3ToBaseEnd, rmBaseToFlange)
        
        r = Rotation.from_matrix(rbLink3EndToFlange)

        angles =  r.as_euler('XYZ',degrees = False) 
        
         #RZ J4
        thetas[3] = angles[0] 
        #RY J5
        thetas[4] = angles[1] 
        #RX J6
        thetas[5] = angles[2]

        for i in range(6):
             thetas[i] = math.degrees(thetas[i])
        
        return thetas

    
     