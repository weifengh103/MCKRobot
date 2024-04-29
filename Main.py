import numpy as np
import time
import matplotlib.pyplot as plt
import matplotlib as mpl
import math

# show decmal in ny matrix
# np.set_printoptions(precision=4)
np.set_printoptions(suppress=True,precision=9)


fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.view_init(elev=0, azim=0, roll=0)

ax = fig.add_subplot()
counter = 1

T01 = np.zeros([4,4])
T02 = np.zeros([4,4])
T03 = np.zeros([4,4])
T04 = np.zeros([4,4])
T05 = np.zeros([4,4])

TJointJiointTrans = [np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4])]
TBaseJiointTrans= [np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4]),np.zeros([4,4])]
#DH parameters
# a = [0,0,50,50,0]
# alphas = [0,90,0,0,90]
# d = [50,0,0,0,50]


# a = [0,50,50,0,0]
# alphas = [math.radians(90),0,0,math.radians(90),0]
# d = [50,0,0,0,50]

#zhang
# a = [0,0,50,0,0]
# alphas = [math.radians(0),math.radians(-90),math.radians(0),math.radians(90),math.radians(-90)]
# d = [50,0,0,50,0]
# theta1 = math.radians( 0 )
# theta2 = math.radians( -90+90  )
# theta3 = math.radians( 90+90)
# theta4 = math.radians( 0)
# theta5 = math.radians( 0)

# 5 DOF
# a = [0,0,50,50,0]
# alphas = [math.radians(0),math.radians(90),math.radians(0),math.radians(0),math.radians(90)]
# d = [50,0,0,0,0]

# web
# a = [0,50,50,50,0]
# alphas = [math.radians(90),math.radians(0),math.radians(0),math.radians(0),math.radians(90)]
# d = [50,0,0,0,0]

#weifeng 
a = [0,50,50,0,0]
alphas = [math.radians(90),math.radians(0),math.radians(0),math.radians(0),math.radians(90)]
d = [50,0,0,0,0]

theta1 = math.radians(0)
theta2 = math.radians(0)
theta3 = math.radians(0)
theta4 = math.radians(0)
theta5 = math.radians(0)
theta6 = math.radians(0)
thetas = [theta1,theta2,theta3,theta4,theta5,theta6]

def getDHTransMatrix(i,thetas,alphas,a,d):
    tempMat = np.matrix([
                    [math.cos(thetas[i]), -math.sin(thetas[i]) * math.cos(alphas[i]),  math.sin(thetas[i]) * math.sin(alphas[i]), a[i] * math.cos(thetas[i])],
                    [math.sin(thetas[i]),  math.cos(thetas[i]) * math.cos(alphas[i]), -math.cos(thetas[i]) * math.sin(alphas[i]), a[i] * math.sin(thetas[i])],
                    [0,                    math.sin(alphas[i]),                        math.cos(alphas[i]),                       d[i]],
                    [0,                    0,                                          0,                                         1]
                    ])
     
    return tempMat

def updateAllTJointJointTrans(thetas,alphas,a,d):
    for i in range(5):
        TJointJiointTrans[i]=getDHTransMatrix(i,thetas,alphas,a,d)
 
def updateAllTBaseJointTrans():
    for i in range(5):
        if i == 0:
            TBaseJiointTrans[i] = TJointJiointTrans[i]
        else:
            TBaseJiointTrans[i] = np.matmul(TBaseJiointTrans[i-1] , TJointJiointTrans[i])
         
def getFKLinkPos():
    global counter
    
    # link length

    # counter = counter +5
    step = 5
    #DH parameters
    # global theta1 
    # global theta2
    # global theta3
    # global theta4
    # global theta5
    global thetas
    
    # theta1 +=   (step/180)*math.pi
    # theta2 +=   (step/180)*math.pi
    # theta3 +=  step/180*math.pi
    # theta4 +=  step/180*math.pi
    

    # thetas = [theta1,theta2,theta3,theta4,theta5]
    
    updateAllTJointJointTrans(thetas,alphas,a,d)
    updateAllTBaseJointTrans()
    
    pBase = np.array([0,0,0,1])
    pShoulder =np.matmul(TBaseJiointTrans[0] , pBase).A1
    pElbow = np.matmul(TBaseJiointTrans[1] , pBase).A1
    pWrist  = np.matmul(TBaseJiointTrans[2] , pBase).A1
 

    xLink1 = [pShoulder[0],pBase[0]]
    yLink1 = [pShoulder[1],pBase[1]]
    zLink1 = [pShoulder[2],pBase[2]]

    xLink2 = [pElbow[0],pShoulder[0]]
    yLink2 = [pElbow[1],pShoulder[1]]
    zLink2 = [pElbow[2],pShoulder[2]]


    xLink3 = [pWrist[0],pElbow[0]]
    yLink3 = [pWrist[1],pElbow[1]]
    zLink3 = [pWrist[2],pElbow[2]]

     
    ax.cla()
    # ax.set_xlim([-100, 100])
    # ax.set_ylim([-100, 100])
    # ax.set_zlim([-100, 100])
    # ax.set_xlabel('$X$' )
    # ax.set_ylabel('$Y$')
    # ax.set_ylabel('$Z$')
    # ax.plot(xLink1,yLink1,zLink1)
    # ax.plot(xLink2,yLink2,zLink2)
    # ax.plot(xLink3,yLink3,zLink3)
    # ax.plot(xLink4,yLink4,zLink4)
    
    
    
    ax.set_xlim([-10, 110])
    ax.set_ylim([-10, 110])
    ax.plot(xLink1,zLink1)
    ax.plot(xLink2,zLink2)
    ax.plot(xLink3,zLink3)


    # plt.show()
    # plt.draw()
    #  ax.fl
    plt.pause(0.00001)
    

    # theta1 +=   (0/180)*math.pi
    # theta2 +=  0/180*math.pi
    # theta3 +=  step/180*math.pi

    # theta4 = math.radians( 0)
    # theta5 = math.radians( 0)
 
def getIKOneToThreeJointAngles(px,py,pz,upPose):
    if(px == 0):
        px = 0.00001
    j1, j2, j3,j4,j5,j6 = 0.0,0.0,0.0,0.0,0.0,0.0
    pzLocal = pz - d[0]
        
    var1 =(a[1]**2 + a[2]**2 - px**2 - pzLocal**2)/(2*a[1]*a[2])

    j3Abs = - math.acos((a[1]**2 + a[2]**2 - px**2 - pzLocal**2)/(2*a[1]*a[2]))
    j3AbsDeg = math.degrees(j3Abs)
    if(upPose == True):
        j3 = - j3Abs
        j2 = math.atan2(pzLocal,px) +  math.atan2(a[2]*math.sin(j3Abs),(a[1] + a[2]*math.cos(j3Abs)))
        
    else:
        j3 = j3Abs
        j2 = math.atan2(pzLocal,px) -  math.atan2(a[2]*math.sin(j3Abs),(a[1] + a[2]*math.cos(j3Abs)))
        
    
    j3AbsDeg = math.degrees(j3Abs)
    j2AbsDeg = math.degrees(j2)
    
    return [j1, j2, j3,j4,j5,j6]

    

def main():
    plt.ion()

    while True:
        global thetas
        thetas = getIKOneToThreeJointAngles(25,0,75,False)
        getFKLinkPos()
    plt.show()
    sss =1

if __name__ == '__main__':
    main()
# EOA link absolut angle

# theta2 solution