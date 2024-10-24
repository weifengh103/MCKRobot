from MCKRobot import MCKRobot as Robot
from RobotPlot import RobotPlot as RP

def main():
    _robot = Robot()
    robotPlot = RP()
   
    # while True:
    #     _robot.JogRobot(1,3)
    #     robotPlot.PlotJointPost(_robot,5)
    #     continue
    pose = [50,0,50,0,0,0]
    step =  5
    while True:
        for i in range(0,2):
            # for j in range(10):
            #     _robot.JogRobotWorld(i,2)
            #     robotPlot.Plot(_robot)
            #     # robotPlot.PlotJointPost(_robot,5)
            #     formatted_position = f"Robot Position (x, y, z, rx, ry, rz): ({_robot.PosRobot[0]:.2f}, {_robot.PosRobot[1]:.2f}, {_robot.PosRobot[2]:.2f},{_robot.PosRobot[3]:.2f}, {_robot.PosRobot[4]:.2f}, {_robot.PosRobot[5]:.2f})"
            #     print(formatted_position)
            for i in range(3,6):
                for j in range(int(360/step)):
                    _robot.JogRobotWorld(i,step)
                    robotPlot.Plot(_robot)
                    # robotPlot.PlotJointPost(_robot,5)
                    formatted_position = f"Robot Position (x, y, z, rx, ry, rz): ({_robot.PosRobot[0]:.2f}, {_robot.PosRobot[1]:.2f}, {_robot.PosRobot[2]:.2f},{_robot.PosRobot[3]:.2f}, {_robot.PosRobot[4]:.2f}, {_robot.PosRobot[5]:.2f})"
                    print(formatted_position)
            
        

    while True:
        for i in range(3,4):
            
            for j in range(8):
                
                # _robot.MoveL(pose)
                
                # pose[i] =step
                robotPlot.Plot(_robot)
                # robotPlot.PlotJointPost(_robot,5)
                formatted_position = f"Robot Position (x, y, z, rx, ry, rz): ({_robot.PosRobot[0]:.2f}, {_robot.PosRobot[1]:.2f}, {_robot.PosRobot[2]:.2f},{_robot.PosRobot[3]:.2f}, {_robot.PosRobot[4]:.2f}, {_robot.PosRobot[5]:.2f})"
                print(formatted_position)

            for j in range(8):
                    
                _robot.MoveL(pose)
                # pose[i] =step
                robotPlot.Plot(_robot)
                # robotPlot.PlotJointPost(_robot,5)
                formatted_position = f"Robot Position (x, y, z, rx, ry, rz): ({_robot.PosRobot[0]:.2f}, {_robot.PosRobot[1]:.2f}, {_robot.PosRobot[2]:.2f},{_robot.PosRobot[3]:.2f}, {_robot.PosRobot[4]:.2f}, {_robot.PosRobot[5]:.2f})"
                print(formatted_position)
        


            # for j in range(dist):
            #     robotPlot.Plot(_robot)
            #     if(i<3):
            #         _robot.JogRobotTCP(2,i)
                 
            #         formatted_position = f"Robot Position (x, y, z, rx, ry, rz): ({_robot.PosRobot[0]:.2f}, {_robot.PosRobot[1]:.2f}, {_robot.PosRobot[2]:.2f},{_robot.PosRobot[3]:.2f}, {_robot.PosRobot[4]:.2f}, {_robot.PosRobot[5]:.2f})"
            #         print(formatted_position)

            #     else:
            #         _robot.JogRobotTCP(2,i)
            #         formatted_position = f"Robot Position (x, y, z, rx, ry, rz): ({_robot.PosRobot[0]:.2f}, {_robot.PosRobot[1]:.2f}, {_robot.PosRobot[2]:.2f},{_robot.PosRobot[3]:.2f}, {_robot.PosRobot[4]:.2f}, {_robot.PosRobot[5]:.2f})"
            #         print(formatted_position)
            # for j in range(dist):
            #     robotPlot.Plot(_robot)
            #     if(i<3):
            #         _robot.JogRobotTCP(-2,i)
            #         formatted_position = f"Robot Position (x, y, z, rx, ry, rz): ({_robot.PosRobot[0]:.2f}, {_robot.PosRobot[1]:.2f}, {_robot.PosRobot[2]:.2f},{_robot.PosRobot[3]:.2f}, {_robot.PosRobot[4]:.2f}, {_robot.PosRobot[5]:.2f})"
            #         print(formatted_position)
            #     else:
            #         _robot.JogRobotTCP(-2,i)
            #         formatted_position = f"Robot Position (x, y, z, rx, ry, rz): ({_robot.PosRobot[0]:.2f}, {_robot.PosRobot[1]:.2f}, {_robot.PosRobot[2]:.2f},{_robot.PosRobot[3]:.2f}, {_robot.PosRobot[4]:.2f}, {_robot.PosRobot[5]:.2f})"
            #         print(formatted_position)



            # j = 0
            # for j in range(10):
            #     _robot.JogJoint(i,2)
            #     MCKPlot()
        # for i in range(100):
        #     MCKPlot()
        #     # _robot.move(x,y,z)
        #     # x-=.5
        #     # y-=.5
        #     z-=1
        # for i in range(100):
        #     MCKPlot()
        #     # _robot.move(x,y,z)
        #     # x-=.5
        #     # y-=.5
        #     # z+=1
        
    # plt.show()
    sss =1

if __name__ == '__main__':
    main()
 