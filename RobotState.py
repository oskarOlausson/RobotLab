"""
File that handles  the states of the robot
"""

import Postman,Trig
from math import atan2,pi

#returns the robots direction in degrees
def getDirection():
    unitVector = Postman.getBearing()
    ux=unitVector['X']
    uy=unitVector['Y']
    return Trig.angleToPoint(0,0,ux,uy)

#returns angular,linear
def getSpeed():
    return Postman.getSpeed()

def getWidth():
    #in reality .4 but marginals
    return .5;

#returns x,y
def getPosition():
    data = Postman.getPose()
    x = data['Pose']['Position']['X']
    y = data['Pose']['Position']['Y']
    return x,y


if __name__ == '__main__':
    a,l=getSpeed()
    print "angular %.3f" % a
    print "linear %.3f" % l