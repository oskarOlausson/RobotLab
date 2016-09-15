"""
File that handles trigonomics math
"""
from math import sin, cos, pi, atan2, sqrt

def sign(a):
    #TODO
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
    laserAngle = int(round(radToDeg(rad)+135-radToDeg(robotDirection)))
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

if __name__ == "__main__":
    a=4
