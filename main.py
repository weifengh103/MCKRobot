from MCKRobot import MCKRobot as Robot
from RobotPlot import RobotPlot as RP
import json
import paho.mqtt.client as mqtt
import json

import time

def main():
    #MQTT
    broker_address = "localhost"
    port = 1883
    tmBaseJiointTopic = "tmBaseJioint"
    client = mqtt.Client()
    client.connect(broker_address, port)

    
    _robot = Robot()
    robotPlot = RP()
   
    # while True:
    #     _robot.JogRobot(1,3)
    #     robotPlot.PlotJointPost(_robot,5)
    #     continue
    pose = [50,0,50,0,0,0]

    stepL = 2
    stepR = 5

    travelL = 20
    travelR = 36

    step = 0
    travel = 0

    while True:
        
        for i in range(0,1):
            for j in range(0,20):
                _robot.JogRobotWorld(i,1)
                robotPlot.Plot(_robot)

                tm1 = _robot.tmBaseJioint[0]
                tm2 = _robot.tmBaseJioint[1]

                tmBaseJoint_as_lists = [matrix.tolist() for matrix in _robot.tmInitCurrBaseJoint]
                tmJson1 =tmBaseJiointTopic+':' + json.dumps(tmBaseJoint_as_lists)
                client.publish(tmBaseJiointTopic, tmJson1)

                print(tm2)
                time.sleep(0.1)
                
            for j in range(0,20):
                _robot.JogRobotWorld(i,-1)
                robotPlot.Plot(_robot)
                
                tm1 = _robot.tmBaseJioint[0]
                tm2 = _robot.tmBaseJioint[1]
                tmBaseJoint_as_lists = [matrix.tolist() for matrix in _robot.tmInitCurrBaseJoint]
                tmJson1 =tmBaseJiointTopic+':' + json.dumps(tmBaseJoint_as_lists)
                client.publish(tmBaseJiointTopic, tmJson1)
                
                print(tm2)
                time.sleep(0.1)


        # for k in range(1,2):
        #     for i in range(0,3):

        #         if(i<3):
        #             travel = travelL
        #             step = stepL
        #         else:
        #             travel = travelR
        #             step = stepR

        #         for _ in range(travel):
        #             if(k == 0):
        #                 _robot.JogRobotWorld(i,step)
        #             else:
        #                 _robot.JogRobotTCP(i,step)
        #             robotPlot.Plot(_robot)
        #             formatted_position = f"Robot Position (x, y, z, rx, ry, rz): ({_robot.PosRobot[0]:.2f}, {_robot.PosRobot[1]:.2f}, {_robot.PosRobot[2]:.2f},{_robot.PosRobot[3]:.2f}, {_robot.PosRobot[4]:.2f}, {_robot.PosRobot[5]:.2f})"
        #             # print(formatted_position)

        #         for _ in range(travel):
        #             if(k == 0):
        #                 _robot.JogRobotWorld(i,-step)
        #             else:
        #                 _robot.JogRobotTCP(i,-step)
        #             robotPlot.Plot(_robot)
        #             formatted_position = f"Robot Position (x, y, z, rx, ry, rz): ({_robot.PosRobot[0]:.2f}, {_robot.PosRobot[1]:.2f}, {_robot.PosRobot[2]:.2f},{_robot.PosRobot[3]:.2f}, {_robot.PosRobot[4]:.2f}, {_robot.PosRobot[5]:.2f})"
        #             # print(formatted_position)

        # for i in range(3,6):
        #     for _ in range(travelR):
        #         _robot.JogJoint(i,stepR)
        #         robotPlot.Plot(_robot)
        #         formatted_position = f"Robot Position (x, y, z, rx, ry, rz): ({_robot.PosRobot[0]:.2f}, {_robot.PosRobot[1]:.2f}, {_robot.PosRobot[2]:.2f},{_robot.PosRobot[3]:.2f}, {_robot.PosRobot[4]:.2f}, {_robot.PosRobot[5]:.2f})"
        #         print(formatted_position)
        #     for _ in range(travelR):
        #         _robot.JogJoint(i,-stepR)
        #         robotPlot.Plot(_robot)
        #         formatted_position = f"Robot Position (x, y, z, rx, ry, rz): ({_robot.PosRobot[0]:.2f}, {_robot.PosRobot[1]:.2f}, {_robot.PosRobot[2]:.2f},{_robot.PosRobot[3]:.2f}, {_robot.PosRobot[4]:.2f}, {_robot.PosRobot[5]:.2f})"
        #         print(formatted_position)

            
def publish_matrix(matrix_json):
    client = mqtt.Client()
    client.connect(broker_address, port)
    client.publish(topic, matrix_json)
    client.disconnect()


if __name__ == '__main__':
    main()
 