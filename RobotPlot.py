
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

        # get TCP display axis position
        xTCPX = [_rb.pDispTCPOrig[0],_rb.pDispTCPX[0]]
        yTCPX = [_rb.pDispTCPOrig[1],_rb.pDispTCPX[1]]
        zTCPX = [_rb.pDispTCPOrig[2],_rb.pDispTCPX[2]]
        
        xTCPY = [_rb.pDispTCPOrig[0],_rb.pDispTCPY[0]]
        yTCPY = [_rb.pDispTCPOrig[1],_rb.pDispTCPY[1]]
        zTCPY = [_rb.pDispTCPOrig[2],_rb.pDispTCPY[2]]
        
        xTCPZ = [_rb.pDispTCPOrig[0],_rb.pDispTCPZ[0]]
        yTCPZ = [_rb.pDispTCPOrig[1],_rb.pDispTCPZ[1]]
        zTCPZ = [_rb.pDispTCPOrig[2],_rb.pDispTCPZ[2]]
        

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
        
        self.ax.plot(xTCPX,yTCPX,zTCPX,color="red",linewidth = '3')
        self.ax.plot(xTCPY,yTCPY,zTCPY,color="green",linewidth = '3')
        self.ax.plot(xTCPZ,yTCPZ,zTCPZ,color="blue",linewidth = '3')
        
        plt.pause(0.00001)
        