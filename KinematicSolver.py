import numpy as np
import time
import matplotlib.pyplot as plt
import matplotlib as mpl
import math

class KinematicSolver:
    
    _TJointJiointTrans = [np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4])]
    _TBaseJiointTrans= [np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4])]
    
    def __init__(self, a, alpha, d):
        self.a = a
        self.alpha = alpha
        self.d = d

    # FK Section
    
    def getDHTransMatrix(i,thetas,alphas,a,d):
        tempMat = np.matrix([
                        [math.cos(thetas[i]), -math.sin(thetas[i]) * math.cos(alphas[i]),  math.sin(thetas[i]) * math.sin(alphas[i]), a[i] * math.cos(thetas[i])],
                        [math.sin(thetas[i]),  math.cos(thetas[i]) * math.cos(alphas[i]), -math.cos(thetas[i]) * math.sin(alphas[i]), a[i] * math.sin(thetas[i])],
                        [0,                    math.sin(alphas[i]),                        math.cos(alphas[i]),                       d[i]],
                        [0,                    0,                                          0,                                         1]
                        ])
        
        return tempMat

    def updateAllTJointJointTrans(self,thetas,alphas,a,d):
        for i in range(5):
            self.TJointJiointTrans[i]=self.getDHTransMatrix(i,thetas,alphas,a,d)
    
    def updateAllTBaseJointTrans(self):
        for i in range(5):
            if i == 0:
                self.TBaseJiointTrans[i] = self.TJointJiointTrans[i]
            else:
                self.BaseJiointTrans[i] = np.matmul(self.TBaseJiointTrans[i-1] , self.TJointJiointTrans[i])
                
    def UpdateFK(self,thetas,alphas,a,d,pBase,pShoulder,pElbow,pWrist):
        
        self.updateAllTJointJointTrans(thetas,alphas,a,d)
        self.updateAllTBaseJointTrans()
        
        pBase = np.array([0,0,0,1])
        pShoulder =np.matmul(self._TBaseJiointTrans[0] , pBase).A1
        pElbow = np.matmul(self._TBaseJiointTrans[1] , pBase).A1
        pWrist  = np.matmul(self._TBaseJiointTrans[2] , pBase).A1

        #TODO Temperately
        pTCP = pWrist
        # xLink1 = [pShoulder[0],pBase[0]]
        # yLink1 = [pShoulder[1],pBase[1]]
        # zLink1 = [pShoulder[2],pBase[2]]

        # xLink2 = [pElbow[0],pShoulder[0]]
        # yLink2 = [pElbow[1],pShoulder[1]]
        # zLink2 = [pElbow[2],pShoulder[2]]


        # xLink3 = [pWrist[0],pElbow[0]]
        # yLink3 = [pWrist[1],pElbow[1]]
        # zLink3 = [pWrist[2],pElbow[2]]

 

    
 
 
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
        
        pxRotated, j1 = self.getRoatedXandTheata1(px,py)
        
        
        if(pxRotated == 0):
            pxRotated = 0.00001

        pzLocal = pz - d[0]
            


        j3Abs =  math.pi - math.acos((a[1]**2 + a[2]**2 - pxRotated**2 - pzLocal**2)/(2*a[1]*a[2])) 
        
        # j3Abs = j3Abs + math.pi/2
        j3AbsDeg = math.degrees(j3Abs)
        if(elbowUp == True):
            j3 = - j3Abs
            j2 = math.atan(pzLocal/pxRotated) +  math.atan(a[2]*math.sin(j3Abs)/(a[1] + a[2]*math.cos(j3Abs)))
            
        else:
            j3 = j3Abs
            j2 = math.atan(pzLocal/pxRotated) -  math.atan(a[2]*math.sin(j3Abs)/(a[1] + a[2]*math.cos(j3Abs)))
            
        
        j3AbsDeg = math.degrees(j3Abs)
        j2AbsDeg = math.degrees(j2)
        
        return [j1, j2, j3,j4,j5,j6]
    
     