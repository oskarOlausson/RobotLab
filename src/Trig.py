"""
File that handles trigonomics math
"""
from math import sin, cos, pi, atan2, sqrt
from Postman import getLaser, postSpeed
import RobotState
import time

def sign(a):
    return int(a>0) - int(a<0)

#Converts from radians to degrees
def radToDeg(rad):
    return rad * (180 /pi)

#converts drom degrees to radians
def degToRad(deg):
    return deg * (pi /180)

#takes an angle (in degrees) and returns the laser that points in that direction
#...hopefully (in the test it seems to work
def radToLaser(rad, robotDirection):
    #direction according to robot
    rad -= robotDirection
    rad %= 2*pi
    if rad>pi: rad -= 2*pi
    #angle between -pi and pi
    laserAngle = int(round(radToDeg(rad)+135))
    if laserAngle>270 or laserAngle<0: laserAngle=-1
    return laserAngle

def getCenterOfTurn(r, robotAngle, x, y):

    cx = x + r * cos(robotAngle + (pi / 2))
    cy = y + r * sin(robotAngle + (pi / 2))
    return cx, cy

def distanceToPoint(x,y,goalx,goaly):
    return sqrt((goalx-x) ** 2 + (goaly-y) ** 2 )

def straightLength(length,direction):
    return cos(direction)*length

def angleToPoint(x,y,goalx,goaly):
    return atan2(goaly-y, goalx-x)

def angleDifference(angle,goalAngle):
    a = angle - goalAngle
    a = ((a + pi) % (pi*2)) - pi
    return abs(a)

def angleDifferenceDirection(startAngle, endAngle,turnDir):
    startAngle %= 2 * pi
    endAngle %= 2 * pi
    diff = (endAngle-startAngle) % (2 * pi)

    if turnDir==1: return diff
    else: return (2*pi-diff) % (2*pi)


if __name__ == "__main__":
    _=0

