"""
File that handles trigonomics math
"""


import Postman
import RobotState
from RobotMove import turnDirection
from math import cos, pi, atan2, sqrt

#Converts from radians to degrees
def radToDeg(rad):
    return rad * (180 /pi)

#converts drom degrees to radians
def degToRad(deg):
    return deg * (pi /180)

#takes an angle (in degrees) and returns the laser that points in that direction
#...hopefully (in the test it seems to work
def degToLaser(deg,robotDirection):
    laserAngle = int(round(deg+135-robotDirection))
    return laserAngle

def distanceToPoint(x,y,goalx,goaly):
    return sqrt((goalx-x) ** 2 + ( goaly-y) ** 2 )

def straightLength(length,direction):
    return cos(degToRad(direction))*length

def angleToPoint(x,y,goalx,goaly):
    return radToDeg(atan2(goaly-y, goalx-x))

def angleDifference(angle,goalAngle):
    a = angle - goalAngle
    a = ((a + 180) % 360) - 180
    return abs(a)

if __name__ == "__main__":
    a=4
