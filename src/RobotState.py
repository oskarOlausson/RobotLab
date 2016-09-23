"""
File that handles  the states of the robot
"""

from math import sin,cos,pi,sqrt

import Postman
import Trig

#returns the robots direction in degrees
def getDirection():
    unitVector = Postman.getBearing()
    ux=unitVector['X']
    uy=unitVector['Y']
    return Trig.angleToPoint(0, 0, ux, uy)

#returns angular,linear
def getSpeed():
    return Postman.getSpeed()

def getWidth():
    #in reality .4 but marginals
    return .5

#returns x,y
def getPosition():
    data = Postman.getPose()
    x = data['Pose']['Position']['X']
    y = data['Pose']['Position']['Y']
    return x,y

def getLaserLength(laser, angle, robotAngle):
    index = Trig.radToLaserFloat(angle, robotAngle)
    leftIndex = int(index)
    rightIndex = leftIndex + 1

    if (rightIndex<0 or leftIndex>270): dist = sqrt(2)*(getSize()/2)
    elif leftIndex==270: dist = laser['Echoes'][270]
    elif rightIndex==0: dist = laser['Echoes'][0]
    else:
        diffL = abs(index-leftIndex)
        diffR = abs(index-rightIndex)

        distL = laser['Echoes'][leftIndex]
        distR = laser['Echoes'][rightIndex]

        dist = (distL*diffR) + (distR*diffL)

    return dist


def getCorners(x, y, angle, cornerNumber):
    len = getSize() / sqrt(2)

    #0 is upperLeft, 1 is lowerLeft, 2 is lowerRight and 3 is upperRight
    cornerNumber= (pi/2) * (cornerNumber % 4) + (pi/4)

    cx = x + len * cos(angle + cornerNumber)
    cy = y + len * sin(angle + cornerNumber)

    return cx,cy


def getSides(x, y, angle, sideNumber):
    len = getSize()/2

    # 0 is up, 1 is left, 2 is down and 3 is right
    sideNumber = (pi/2) * (sideNumber % 4)

    cx = x + len * cos(angle + sideNumber)
    cy = y + len * sin(angle + sideNumber)

    return cx, cy

def getBoth(gx,gy,angle,number):
    if number % 2 == 0:
        number/=2
        return getSides(gx,gy,angle,number)
    else:
        number-=1
        number/=2
        return getCorners(gx,gy,angle,number)

#TODO this should probably be 0.8
def getSize():
    return 0.65

def getActualSize():
    return 0.40000000596046448

if __name__ == '__main__':
    getLaserLength(None,0,0)


def getLaserLengthFromIndex(laser, index):
    if (index<0 or index>270): return sqrt(2)*(getSize()/2)
    else: return laser['Echoes'][index]