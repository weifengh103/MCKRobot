import numpy as np
import time
import matplotlib.pyplot as plt
import matplotlib as mpl
import math

from MCKRobot import MCKRobot as rb
# show decmal in ny matrix
# np.set_printoptions(precision=4)



np.set_printoptions(suppress=True,precision=9)


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
 

#weifeng 
# a = [0,50,50,0,0]
# alphas = [math.radians(90),math.radians(0),math.radians(0),math.radians(0),math.radians(90)]
# d = [50,0,0,0,0]
 
_rb = rb()
         
def MCKPlot():
 
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

     
    ax.cla()
    
    ax.set_xlim([-100, 100])
    ax.set_ylim([-100, 100])
    ax.set_zlim([-100, 100])
    ax.set_xlabel('$X$' )
    ax.set_ylabel('$Y$')
    ax.set_ylabel('$Z$')
    ax.plot(xLink1,yLink1,zLink1)
    ax.plot(xLink2,yLink2,zLink2)
    ax.plot(xLink3,yLink3,zLink3)
    ax.plot(xLink4,yLink4,zLink4)
    ax.plot(xLink5,yLink5,zLink5)
    ax.plot(xLink6,yLink6,zLink6)
    
    
    # ax.set_xlim([-10, 150])
    # ax.set_ylim([-10, 150])
    # ax.plot(xLink1,zLink1)
    # ax.plot(xLink2,zLink2)
    # ax.plot(xLink3,zLink3)


  
    plt.pause(0.00001)
    

    # theta1 +=   (0/180)*math.pi
    # theta2 +=  0/180*math.pi
    # theta3 +=  step/180*math.pi

    # theta4 = math.radians( 0)
    # theta5 = math.radians( 0)
 
def main():
    # plt.ion()
    # x = 85.355
    # y = 0
    # z = 85.355
    
    x = 40
    y = 0
    z = 85.355

    _rb.InitRobot()
    
    # v1,x2 =  getRoatedXandTheata1(-1,1)
    while True:
        for i in range(100):
            MCKPlot()
            # _rb.move(x,y,z)
            # x-=.5
            # y-=.5
            z-=1
        for i in range(100):
            MCKPlot()
            # _rb.move(x,y,z)
            # x-=.5
            # y-=.5
            # z+=1
        
    # plt.show()
    sss =1

if __name__ == '__main__':
    main()
# EOA link absolut angle

# theta2 solution