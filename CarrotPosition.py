

import json, RobotControl
from math import sin, cos, pi, atan2, sqrt


def load(fileString):
    with open(fileString,'r') as data_file:
        data = json.load(data_file)
    return data

def radToDeg(deg):
    return deg * (180/pi)

def robotCanSee(goalx,goaly):
    canSee=False

    angle = int (radToDeg(atan2(goaly-y,goalx-x)))
    distance = sqrt((goalx-x)**2 + (goaly-y)**2)
    laserData=RobotControl.getLaser()

    print 'Distance to point in direction %.3f: %.3f' % (angle,distance)
    print 'Distance to wall in same direction %.3f' % laserData['Echoes'][angle]


    return canSee


if __name__ == '__main__':
    data = load("Path-to-bed.json")

    print 'First point of x-data is %.3f' % (data[0]['Pose']['Position']['X'])
    print 'First point of y-data is %.3f' % (data[0]['Pose']['Position']['Y'])
    print 'Last point of x-data is %.3f' % (data[len(data)-1]['Pose']['Position']['X'])
    print 'Last point of y-data is %.3f' % (data[len(data)-1]['Pose']['Position']['Y'])

    x,y = RobotControl.getPosition()

    print 'Robots x-position is %.3f' % x
    print 'Robots x-position is %.3f' % y

    robotCanSee(data[300]['Pose']['Position']['X'],data[300]['Pose']['Position']['Y'])

