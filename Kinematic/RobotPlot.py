
import matplotlib.pyplot as plt
import matplotlib as mpl
from MCKRobot import MCKRobot as robot
import numpy as np


class RobotPlot:

    np.set_printoptions(suppress=True,precision=9)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    def Plot(self,_rb:robot ):
        # get all link positions
        xLink1 = [_rb.pShoulder[0],_rb.pBase[0]]
        yLink1 = [_rb.pShoulder[1],_rb.pBase[1]]
        zLink1 = [_rb.pShoulder[2],_rb.pBase[2]]

        xLink2 = [_rb.pElbow[0],_rb.pShoulder[0]]
        yLink2 = [_rb.pElbow[1],_rb.pShoulder[1]]
        zLink2 = [_rb.pElbow[2],_rb.pShoulder[2]]

        xLink3 = [_rb.pElbow2[0],_rb.pElbow[0]]
        yLink3 = [_rb.pElbow2[1],_rb.pElbow[1]]
        zLink3= [_rb.pElbow2[2],_rb.pElbow[2]]

        xLink4 = [_rb.pWristPos[0],_rb.pElbow2[0]]
        yLink4 = [_rb.pWristPos[1],_rb.pElbow2[1]]
        zLink4 = [_rb.pWristPos[2],_rb.pElbow2[2]]
        
        xLink5 = [_rb.pFlange[0],_rb.pWristPos[0]]
        yLink5 = [_rb.pFlange[1],_rb.pWristPos[1]]
        zLink5 = [_rb.pFlange[2],_rb.pWristPos[2]]
        
        xLink6 = [_rb.pTCPPos[0],_rb.pFlange[0]]
        yLink6 = [_rb.pTCPPos[1],_rb.pFlange[1]]
        zLink6 = [_rb.pTCPPos[2],_rb.pFlange[2]]

        xTCPX = [_rb.pDispTCP[0][0],_rb.pDispTCP[1][0]]
        yTCPX = [_rb.pDispTCP[0][1],_rb.pDispTCP[1][1]]
        zTCPX = [_rb.pDispTCP[0][2],_rb.pDispTCP[1][2]]

        xTCPY = [_rb.pDispTCP[0][0],_rb.pDispTCP[2][0]]
        yTCPY = [_rb.pDispTCP[0][1],_rb.pDispTCP[2][1]]
        zTCPY = [_rb.pDispTCP[0][2],_rb.pDispTCP[2][2]]


        xTCPZ = [_rb.pDispTCP[0][0],_rb.pDispTCP[3][0]]
        yTCPZ = [_rb.pDispTCP[0][1],_rb.pDispTCP[3][1]]
        zTCPZ = [_rb.pDispTCP[0][2],_rb.pDispTCP[3][2]]


        # Joint or Flange
        index = 5
        length = 20
        xEnd = np.matmul(_rb.tmBaseJioint[index],[length,0,0,1] ).A1
        yEnd = np.matmul( _rb.tmBaseJioint[index],[0,length,0,1]).A1
        zEnd = np.matmul( _rb.tmBaseJioint[index],[0,0,length,1]).A1
        jOrig = np.transpose(_rb.tmBaseJioint[index][:3,3]).A1
  
        xTCPX2 = [jOrig[0],xEnd[0]]
        yTCPX2= [jOrig[1],xEnd[1]]
        zTCPX2 = [jOrig[2],xEnd[2]]

        xTCPY2 = [jOrig[0],yEnd[0]]
        yTCPY2 = [jOrig[1],yEnd[1]]
        zTCPY2 = [jOrig[2],yEnd[2]]

        xTCPZ2 = [jOrig[0],zEnd[0]]
        yTCPZ2 = [jOrig[1],zEnd[1]]
        zTCPZ2 = [jOrig[2],zEnd[2]]


        # clear graph and plot all 
        self.ax.cla()
        scale = 80
        
        axisOffset = 50
        self.ax.set_xlim([-scale+axisOffset, scale+axisOffset])
        self.ax.set_ylim([-scale, scale])
        self.ax.set_zlim([-scale + axisOffset, scale+axisOffset])
        self.ax.set_xlabel('$X$' )
        self.ax.set_ylabel('$Y$')
        self.ax.set_zlabel('$Z$')
        self.ax.plot(xLink1,yLink1,zLink1,color="black",linewidth = '3')
        self.ax.plot(xLink2,yLink2,zLink2,color="black",linewidth = '3')
        self.ax.plot(xLink3,yLink3,zLink3,color="black",linewidth = '3')
        self.ax.plot(xLink4,yLink4,zLink4,color="black",linewidth = '3')
        self.ax.plot(xLink5,yLink5,zLink5,color="black",linewidth = '3')
        self.ax.plot(xLink6,yLink6,zLink6,color="black",linewidth = '3')
      
        self.ax.plot(xTCPX2,yTCPX2,zTCPX2,color="red",linewidth = '3')
        self.ax.plot(xTCPY2,yTCPY2,zTCPY2,color="green",linewidth = '3')
        self.ax.plot(xTCPZ2,yTCPZ2,zTCPZ2,color="blue",linewidth = '3')

        showTCP = True
        if(showTCP):
            self.ax.plot(xTCPX,yTCPX,zTCPX,color="red",linewidth = '3')
            self.ax.plot(xTCPY,yTCPY,zTCPY,color="green",linewidth = '3')
            self.ax.plot(xTCPZ,yTCPZ,zTCPZ,color="blue",linewidth = '3')
        
        plt.pause(0.00001)
    
