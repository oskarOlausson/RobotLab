"""
File that handles  the states of the robot
"""

from math import sin,cos,pi,sqrt

import Postman
import Trig


def getDirection():
    """
        :return: The heading of the robot as radians from straight up
    """
    unitVector = Postman.getBearing()
    ux=unitVector['X']
    uy=unitVector['Y']
    return Trig.angleToPoint(0, 0, ux, uy)

def getPosition():
    """
        :return: the position of the robot in the global positionsystem
    """
    data = Postman.getPose()
    x = data['Pose']['Position']['X']
    y = data['Pose']['Position']['Y']
    return x,y

def getLaserLength(laser, angle, robotAngle):
    """
        Returns the laserlength of a certain angle,
         if the angle is between two lasers, a weighted average is used
         to derive a more exact distance
         uses: radToLaserFloat
    """
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
    """
        :param x, y: the position you want to investigate
        :param angle: the angle the robot would be
        :param cornerNumber: 0 for upper left, 1 for lower left, 2 for lower right, 3 for upper right
        :return: the x and y position of the corner
    """
    len = getSize() / sqrt(2)

    #0 is upperLeft, 1 is lowerLeft, 2 is lowerRight and 3 is upperRight
    cornerNumber= (pi/2) * (cornerNumber % 4) + (pi/4)

    cx = x + len * cos(angle + cornerNumber)
    cy = y + len * sin(angle + cornerNumber)

    return cx,cy


def getSize():
    """
        Returns the size of the robot plus some padding so small variance wont crash it in to corners
    """
    return 0.75

def getActualSize():
    """
        :return the actual size of robot
    """
    return 0.40000000596046448

def getLaserLengthFromIndex(laser, index):
    """
        Returns the length of a laser from an index, if outside range, assume length of robot at least
    """
    if (index<0 or index>270): return (getSize()/2)
    else: return laser['Echoes'][index]