import numpy as np
import time
import matplotlib.pyplot as plt
import matplotlib as mpl
import math
from scipy.spatial.transform import Rotation   

class KinematicSolver:
    
    _TJointJiointTrans = [np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4])]
    _TBaseJiointTrans = [np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4])]
 
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

    def updateAllTJointJointTrans(self,thetas,alphas,a,d):
        # Update first four joint to joint transition matrix based on DH Matrix
        
        for i in range(6):
            # When i = 0, first matrix is from base to joint 2 (Shoulder)
            self._TJointJiointTrans[i]=self.getDHTransMatrix(i,thetas,alphas,a,d)
            
        # # Treat joint 4 as a sphere joint with  3 rotational degree. Calculate base on rotation sequence x'→y'→z' 
        # # Reference: https://www.mecademic.com/academic_articles/space-orientation-euler-angles/
        
        # # assign  rx ry rz based on coordination on joint 4
        # # rx = thetas[5]
        # # ry = thetas[4] 
        # # rz = thetas[3]
        # rx = math.radians(0)
        # ry = math.radians(0)
        # rz = math.radians(45)
        # # Construct trans matrix from joint 4 to TCP tip
        # Rx = np.matrix([[1, 0, 0], [0, np.cos(rx), -np.sin(rx)], [0, np.sin(rx), np.cos(rx)]])
        # Ry = np.matrix([[np.cos(ry), 0, np.sin(ry)], [0, 1, 0], [-np.sin(ry), 0, np.cos(ry)]])
        # Rz = np.matrix([[np.cos(rz), -np.sin(rz), 0], [np.sin(rz), np.cos(rz), 0], [0, 0, 1]])
        
        
        # Rxyz = np.matmul(np.matmul(Rx,Ry),Rz)
        
        # #TODO: Set fixed TCP for now
        # pTCP = [0,1,0]
        # Txyz =np.zeros((4, 4))
        # # pTCPRotated = np.dot(Rxyz,pTCP)
        # Txyz[:3, :3] = Rxyz
        # Txyz[:3, 3] = pTCP
        # Txyz[3, 3] = 1      
        # self._TJointJiointTrans[3] =  np.matrix(Txyz)
        
        # pTCP2 = [30,0,0,0]
        # pTCPRotated = np.dot(self._TJointJiointTrans[3],pTCP2) 
        pass
        
     
    
    def updateAllTBaseJointTrans(self):
         # Update first four base to joint transition matrix based on DH Matrix
        for i in range(6):
            if i == 0:
                self._TBaseJiointTrans[i] = self._TJointJiointTrans[i]
            else:
                temp =np.dot(self._TBaseJiointTrans[i-1] , self._TJointJiointTrans[i])
                self._TBaseJiointTrans[i] = np.matmul(self._TBaseJiointTrans[i-1] , self._TJointJiointTrans[i])
        pass
                
    def UpdateFK(self,thetas,alphas,a,d,pJoints,pDispTCP):
 
        self.updateAllTJointJointTrans(thetas,alphas,a,d)
        self.updateAllTBaseJointTrans()
        

        pJoints[1] = np.matmul(self._TBaseJiointTrans[0] , pJoints[0]).A1
        pJoints[2] = np.matmul(self._TBaseJiointTrans[1] , pJoints[0]).A1
        pJoints[3] = np.matmul(self._TBaseJiointTrans[2] , pJoints[0]).A1
        pJoints[4]=np.matmul(self._TBaseJiointTrans[3] , pJoints[0]).A1
        pJoints[5]=np.matmul(self._TBaseJiointTrans[4] , pJoints[0]).A1
        pJoints[6]=np.matmul(self._TBaseJiointTrans[5] , pJoints[0]).A1


        dispTCPAxisLength = 20
        pDispTCP[0] = np.matmul(self._TBaseJiointTrans[5] , [0,0,0,1]).A1
        pDispTCP[1] = np.matmul(self._TBaseJiointTrans[5] , [dispTCPAxisLength,0,0,1]).A1
        pDispTCP[2] = np.matmul(self._TBaseJiointTrans[5] , [0,dispTCPAxisLength,0,1]).A1
        pDispTCP[3] = np.matmul(self._TBaseJiointTrans[5] , [0,0,dispTCPAxisLength,1]).A1
        
        
            
            
        #TODO Temperately PTCP = Pwirst
        # pJoints[4]=np.matmul(self._TBaseJiointTrans[3] , pJoints[0]).A1
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

 

    def updateEulerAngles(self):
   
        rmTCP =  self._TBaseJiointTrans[5] [:3, :3]
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
    
     