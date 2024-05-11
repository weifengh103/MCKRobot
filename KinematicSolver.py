import numpy as np
import time
import matplotlib.pyplot as plt
import matplotlib as mpl
import math
from scipy.spatial.transform import Rotation   

class KinematicSolver:
    

 
    def __init__(self):
        pass
     
    # FK Section
    
    def getDHTransMatrix(self,i,thetas,alphas,a,d):
        tempMat = np.matrix([
                        [math.cos(thetas[i]), -math.sin(thetas[i]) * math.cos(alphas[i]),  math.sin(thetas[i]) * math.sin(alphas[i]), a[i] * math.cos(thetas[i])],
                        [math.sin(thetas[i]),  math.cos(thetas[i]) * math.cos(alphas[i]), -math.cos(thetas[i]) * math.sin(alphas[i]), a[i] * math.sin(thetas[i])],
                        [0,                    math.sin(alphas[i]),                        math.cos(alphas[i]),                       d[i]],
                        [0,                    0,                                          0,                                         1]
                        ])
        
        return tempMat

    def updateAllTJointJointTrans(self,tmJointJoint,thetas,alphas,a,d):
        # Update first four joint to joint transition matrix based on DH Matrix
        
        # Note: When i = 0, first matrix is from base to joint 2 (Shoulder)
        for i in range(6):
            tmJointJoint[i]=self.getDHTransMatrix(i,thetas,alphas,a,d)
    
    def updateAllTBaseJointTrans(self,tmBaseJioint,tmJointJoint):
         # Update first four base to joint transition matrix based on DH Matrix
        for i in range(6):
            if i == 0:
                tmBaseJioint[i] = tmJointJoint[i]
            else:
                temp =np.dot(tmBaseJioint[i-1] , tmJointJoint[i])
                tmBaseJioint[i] = np.matmul(tmBaseJioint[i-1] , tmJointJoint[i])
                
    def UpdateFK(self,tmBaseJioint,tmJointJoint,thetas,alphas,a,d,pJoints,pDispTCP):
 
        self.updateAllTJointJointTrans(tmJointJoint,thetas,alphas,a,d)
        self.updateAllTBaseJointTrans(tmBaseJioint,tmJointJoint)
        

        pJoints[1] = np.matmul(tmBaseJioint[0] , pJoints[0]).A1
        pJoints[2] = np.matmul(tmBaseJioint[1] , pJoints[0]).A1
        pJoints[3] = np.matmul(tmBaseJioint[2] , pJoints[0]).A1
        pJoints[4]=np.matmul(tmBaseJioint[3] , pJoints[0]).A1
        pJoints[5]=np.matmul(tmBaseJioint[4] , pJoints[0]).A1
        pJoints[6]=np.matmul(tmBaseJioint[5] , pJoints[0]).A1


        dispTCPAxisLength = 20
        pDispTCP[0] = np.matmul(tmBaseJioint[5] , [0,0,0,1]).A1
        pDispTCP[1] = np.matmul(tmBaseJioint[5] , [dispTCPAxisLength,0,0,1]).A1
        pDispTCP[2] = np.matmul(tmBaseJioint[5] , [0,dispTCPAxisLength,0,1]).A1
        pDispTCP[3] = np.matmul(tmBaseJioint[5] , [0,0,dispTCPAxisLength,1]).A1
        
        
            
            
        #TODO Temperately PTCP = Pwirst
        # pJoints[4]=np.matmul(tmBaseJioint[3] , pJoints[0]).A1
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

 

    def updateEulerAngles(self,tmBaseJioint):
        
        # # Treat joint 4 -6 as a sphere joint with  3 rotational degree. Calculate base on rotation sequence x'→y'→z' 
        # # Reference: https://www.mecademic.com/academic_articles/space-orientation-euler-angles/
   
        rmTCP =  tmBaseJioint[5] [:3, :3]
        r =  Rotation.from_matrix(rmTCP)
        angles = r.as_euler("xyz",degrees=True)

        #### Modify the angles
        print(angles)
       
   
                
        
 
    # IK Section
    def getRoatedXandTheata1(self,px,py):

        theta1 = math.atan2(py,px)
        xAxisAngleDeg = math.degrees(theta1)
        rotatedX = math.pow((py**2 + px**2),0.5)
        
        if(abs(theta1)>math.pi/2):
            return -rotatedX, theta1
        else:
            return rotatedX, theta1
        
    def UpdateIKOneToThreeJoints(self,px,py,pz,elbowUp, thetas, alphas, a, d):
 
        j1 = thetas[0]
        j2 = thetas[1]
        j3 = thetas[2]
        j4 = thetas[3]
        j5 = thetas[4]
        j6 = thetas[5]
        
        pxRotated, thetas[0] = self.getRoatedXandTheata1(px,py)
        
        
        if(pxRotated == 0):
            pxRotated = 0.00001

        pzLocal = pz - d[0]
            
        # pzLocal=0

        j3Abs =  math.pi - math.acos((a[1]**2 + a[3]**2 - pxRotated**2 - pzLocal**2)/(2*a[1]*a[3])) 
        
        # j3Abs = j3Abs + math.pi/2
        j3AbsDeg = math.degrees(j3Abs)
        if(elbowUp == True):
            thetas[2] = - j3Abs
            thetas[1] = math.atan(pzLocal/pxRotated) +  math.atan(a[2]*math.sin(j3Abs)/(a[1] + a[3]*math.cos(j3Abs)))
            
        else:
            thetas[2] = j3Abs
            thetas[1] = math.atan(pzLocal/pxRotated) -  math.atan(a[2]*math.sin(j3Abs)/(a[1] + a[3]*math.cos(j3Abs)))
        
        return [j1, j2, j3,j4,j5,j6]
    
     