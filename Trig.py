"""
File that handles trigonomics math
"""


import Postman
import RobotState
from math import cos, pi, atan2, sqrt

#Converts from radians to degrees
def radToDeg(rad):
    return rad * (180 /pi)

#converts drom degrees to radians
def degToRad(deg):
    return deg * (pi /180)

#takes an angle (in degrees) and returns the laser that points in that direction
#...hopefully (in the test it seems to work
def degToLaser(deg):
    laserData = Postman.getLaser()
    robotAngle=RobotState.getDirection()
    # print "current angle being investigatet is %.3f" % deg
    #print "robotAngle is %.3f" % robotAngle
    # there is a 45 degrees difference between the laser and the degrees
    laserAngle = int(round(deg+135-robotAngle))
    #print "robotlaser chosen is %.3f" % laserAngle
    #return laserData['Echoes'][laserAngle]
    return laserAngle

def distanceToPoint(x,y,goalx,goaly):
    return sqrt((goalx-x) ** 2 + ( goaly-y) ** 2 )

def straightLength(length,direction):
    return cos(degToRad(direction))*length

def angleToPoint(x,y,goalx,goaly):
    return radToDeg(atan2(goaly-y, goalx-x))

"""looks for a point and sees if the laser can look that far
if it can't, there is a wall in the way
returns a bool, canSee"""


