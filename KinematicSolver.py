import numpy as np
import time
import matplotlib.pyplot as plt
import matplotlib as mpl
import math
from scipy.spatial.transform import Rotation   

class KinematicSolver:
    
    
    a = [0] * 6
    d = [0] * 6
    alphas = [0 * 6]
    
    def __init__(self,a,d,alphas,thetas):
        self.a = a
        self.d = d
        self.alphas = alphas
        self.thetas = thetas
        pass
     
    # FK Section
    
    def getDHTransMatrix(self,i,inputAngleDeg):
        newTheta = self.thetas[i] + math.radians( inputAngleDeg[i])
        tempMat = np.matrix([
                        [math.cos(newTheta), -math.sin(newTheta) * math.cos(self.alphas[i]),  math.sin(newTheta) * math.sin(self.alphas[i]), self.a[i] * math.cos(newTheta)],
                        [math.sin(newTheta),  math.cos(newTheta) * math.cos(self.alphas[i]), -math.cos(newTheta) * math.sin(self.alphas[i]), self.a[i] * math.sin(newTheta)],
                        [0,                    math.sin(self.alphas[i]),                        math.cos(self.alphas[i]),                       self.d[i]],
                        [0,                    0,                                          0,                                         1]
                        ])
        
        return tempMat

    def updateAllTJointJointTrans(self,tmJointJoint,inputAngleDeg):
        # Note: When i = 0, first matrix is from base to joint 2 (Shoulder)
        for i in range(6):
            tmJointJoint[i]=self.getDHTransMatrix(i,inputAngleDeg)
            
 
    
    def updateAllTBaseJointTrans(self,tmBaseJioint,tmJointJoint):
         # Update first four base to joint transition matrix based on DH Matrix
        for i in range(6):
            if i == 0:
                tmBaseJioint[i] = tmJointJoint[i]
            else:
                temp =np.dot(tmBaseJioint[i-1] , tmJointJoint[i])
                tmBaseJioint[i] = np.matmul(tmBaseJioint[i-1] , tmJointJoint[i])
                
    def UpdateFK(self,tmBaseJioint,tmJointJoint,inputAngleDeg,pJoints,pDispTCP):
 
        self.updateAllTJointJointTrans(tmJointJoint,inputAngleDeg)
        self.updateAllTBaseJointTrans(tmBaseJioint,tmJointJoint)
        

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
        
            
        print(tmBaseJioint[3])
        pass
        
        
        # # xLink1 = [pShoulder[0],pBase[0]]
        # # yLink1 = [pShoulder[1],pBase[1]]
        # # zLink1 = [pShoulder[2],pBase[2]]

        # # xLink2 = [pElbow[0],pShoulder[0]]
        # # yLink2 = [pElbow[1],pShoulder[1]]
        # # zLink2 = [pElbow[2],pShoulder[2]]


        # # xLink3 = [pWrist[0],pElbow[0]]
        # # yLink3 = [pWrist[1],pElbow[1]]
        # # zLink3 = [pWrist[2],pElbow[2]]

 

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
       
 
    # IK Section
    # def getRoatedXandTheata1(self,px,py):

    #     theta1 = math.atan2(py,px)
    #     xAxisAngleDeg = math.degrees(theta1)
        
    #     #  rotatedX is the same as the length of the arm on X axis 
    #     rotatedX = math.pow((py**2 + px**2),0.5)
        
    #     if(abs(theta1)>math.pi/2):
    #         return -rotatedX, theta1
    #     else:
    #         return rotatedX, theta1
        
    def UpdateIK(self,pose,elbowUp):
 
 
        tmJointJoint = [np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4])]
        tmBaseJioint = [np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4])]
        
        thetas = [0,0,0,0,0,0]
        px = pose[0]
        py = pose[1]
        pz = pose[2]

        #project the next new x to current x' axis and get j1
        
        # pxRotated, thetas[0] = self.getRoatedXandTheata1(px,py)
        
        thetas[0] = math.atan2(py,px)
        
        xAxisAngleDeg = math.degrees(thetas[0] )
        
        #  rotatedX is the same as the length of the arm on X axis 
        pxRotated = math.pow((py**2 + px**2),0.5) 
        
        if(abs(thetas[0])>math.pi/2):
            pxRotated = -pxRotated
       
        
        
        # if(pxRotated == 0):
        #     pxRotated = 0.00001

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
        
        # cos_theta = np.cos( -thetas[0])
        # sin_theta = np.sin( -thetas[0])
    
        # # rotation mt for rotatiing pose to ZX plan
        # rmBaseToFlangeRotated = np.array([[cos_theta, -sin_theta, 0],
        #             [sin_theta, cos_theta, 0],
        #             [0, 0, 1]])
        
        # # get world rotation of flage after rotated back to ZX plan
        # rmBaseToFlange = np.matmul(rmBaseToFlange,rmBaseToFlangeRotated)
        
        
        # # update first FK for geting TM base to link3 end 
        
        self.updateAllTJointJointTrans(tmJointJoint,thetas)
        self.updateAllTBaseJointTrans(tmBaseJioint,tmJointJoint)
        
        # rmBaseToLink2End = np.matmul(np.matmul(np.matmul(tmBaseJioint[0],tmBaseJioint[1]),tmBaseJioint[2]),tmBaseJioint[3])[:3,:3]
        # # rmBaseToLink2End =  np.matmul(tmBaseJioint[0],tmBaseJioint[1])[:3,:3]
        # rmBaseToLink2EndInv = np.linalg.inv(rmBaseToLink2End) 
        
       
        
        yRotationAngle =  thetas[1] + thetas[2]
        zRotationAngle = thetas[0]
        
        # cos_theta1 = np.cos(yRotationAngle)
        # sin_theta1 = np.sin(yRotationAngle)
        
        # rmY = np.array([[cos_theta1, 0, sin_theta1],
        #                 [0, 1, 0],
        #                 [-sin_theta1, 0, cos_theta1]])
        
        # cos_theta = np.cos(zRotationAngle)
        # sin_theta = np.sin(zRotationAngle)
        
        # rmZ = np.array([[cos_theta, -sin_theta, 0],
        #                 [sin_theta, cos_theta, 0],
        #                 [0, 0, 1]])
        
        # rmBaseToLink2End = np.matmul(rmY,rmZ)
        rmBaseToLink2End = Rotation.from_euler('XYZ', [0,  yRotationAngle, zRotationAngle], degrees=False).as_matrix()
 
        
        rmBaseToLink2EndInv = np.linalg.inv(rmBaseToLink2End) 
        
        rbLink2EndToFlange = np.matmul(rmBaseToLink2EndInv, rmBaseToFlange)
        
        
        r = Rotation.from_matrix(rbLink2EndToFlange)
        angles =  r.as_euler('XYZ',degrees = False) 
        
        
        # thetas[2] = thetas[2]+ math.radians(90)
        #RZ J4
        thetas[3] = angles[0] + math.radians(180)
        #RY J5
        thetas[4] = angles[1] 
        #RX J6
        thetas[5] = angles[2]
        
        
        for i in range(6):
             thetas[i] = math.degrees(thetas[i])
        
        
        return thetas
        pass

    
     