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

    stepL = 3
    stepR = 5

    travelL = 20
    travelR = 18

    step = 0
    travel = 0

    while True:
        for k in range(0,2):
            for i in range(0,6):
                if(i<3):
                    travel = travelL
                    step = stepL
                else:
                    travel = travelR
                    step = stepR
                for _ in range(travel):
                    if(k == 0):
                        _robot.JogRobotWorld(i,step)
                    else:
                        _robot.JogRobotTCP(i,step)
                    robotPlot.Plot(_robot)
                    formatted_position = f"Robot Position (x, y, z, rx, ry, rz): ({_robot.PosRobot[0]:.2f}, {_robot.PosRobot[1]:.2f}, {_robot.PosRobot[2]:.2f},{_robot.PosRobot[3]:.2f}, {_robot.PosRobot[4]:.2f}, {_robot.PosRobot[5]:.2f})"
                    print(formatted_position)
                for _ in range(travel):
                    if(k == 0):
                        _robot.JogRobotWorld(i,-step)
                    else:
                        _robot.JogRobotTCP(i,-step)
                    robotPlot.Plot(_robot)
                    formatted_position = f"Robot Position (x, y, z, rx, ry, rz): ({_robot.PosRobot[0]:.2f}, {_robot.PosRobot[1]:.2f}, {_robot.PosRobot[2]:.2f},{_robot.PosRobot[3]:.2f}, {_robot.PosRobot[4]:.2f}, {_robot.PosRobot[5]:.2f})"
                    print(formatted_position)

if __name__ == '__main__':
    main()
 