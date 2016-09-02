"""
File that handles  the states of the robot
"""

import Postman
from math import atan2,pi


#the bearing of the robot in degrees
def getAngle():
    return angle

def updateAngle():
    unitVector = Postman.getBearing()
    ux=unitVector['X']
    uy=unitVector['Y']
    return (atan2(uy ,ux))*(180/pi)

def updatePosition():
    data = Postman.getPose()
    x = data['Pose']['Position']['X']
    y = data['Pose']['Position']['Y']
    return x,y

def getPosition():
    return x,y


x,y=updatePosition()
angle=updateAngle()