import numpy as np
import time
import matplotlib.pyplot as plt
import matplotlib as mpl
import math

class KinematicSolver:
    
    _TJointJiointTrans = [np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4])]
    _TBaseJiointTrans = [np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4])]
 
    def __init__(self, name):
        self.name = name
     
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
        for i in range(5):
            self._TJointJiointTrans[i]=self.getDHTransMatrix(i,thetas,alphas,a,d)
    
    def updateAllTBaseJointTrans(self):
        for i in range(5):
            if i == 0:
                self._TBaseJiointTrans[i] = self._TJointJiointTrans[i]
            else:
                temp =np.dot(self._TBaseJiointTrans[i-1] , self._TJointJiointTrans[i])
                self._TBaseJiointTrans[i] = np.matmul(self._TBaseJiointTrans[i-1] , self._TJointJiointTrans[i])
                
    def UpdateFK(self,thetas,alphas,a,d,pJoints):
 
        self.updateAllTJointJointTrans(thetas,alphas,a,d)
        self.updateAllTBaseJointTrans()
        
     
        pJoints[1] =np.matmul(self._TBaseJiointTrans[0] , pJoints[0]).A1
        pJoints[2] = np.matmul(self._TBaseJiointTrans[1] , pJoints[0]).A1
        pJoints[3]  = np.matmul(self._TBaseJiointTrans[2] , pJoints[0]).A1



        #TODO Temperately PTCP = Pwirst
        pJoints[4]= pJoints[3]
        
        
        # # xLink1 = [pShoulder[0],pBase[0]]
        # # yLink1 = [pShoulder[1],pBase[1]]
        # # zLink1 = [pShoulder[2],pBase[2]]

        # # xLink2 = [pElbow[0],pShoulder[0]]
        # # yLink2 = [pElbow[1],pShoulder[1]]
        # # zLink2 = [pElbow[2],pShoulder[2]]


        # # xLink3 = [pWrist[0],pElbow[0]]
        # # yLink3 = [pWrist[1],pElbow[1]]
        # # zLink3 = [pWrist[2],pElbow[2]]

 

    
 
 
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

        j3Abs =  math.pi - math.acos((a[1]**2 + a[2]**2 - pxRotated**2 - pzLocal**2)/(2*a[1]*a[2])) 
        
        # j3Abs = j3Abs + math.pi/2
        j3AbsDeg = math.degrees(j3Abs)
        if(elbowUp == True):
            thetas[2] = - j3Abs
            thetas[1] = math.atan(pzLocal/pxRotated) +  math.atan(a[2]*math.sin(j3Abs)/(a[1] + a[2]*math.cos(j3Abs)))
            
        else:
            thetas[2] = j3Abs
            thetas[1] = math.atan(pzLocal/pxRotated) -  math.atan(a[2]*math.sin(j3Abs)/(a[1] + a[2]*math.cos(j3Abs)))
            
        
        j3AbsDeg = math.degrees(j3Abs)
        j2AbsDeg = math.degrees(j2)
        
        return [j1, j2, j3,j4,j5,j6]
    
     