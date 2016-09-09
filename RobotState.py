"""
File that handles  the states of the robot
"""

import Postman,Trig
from math import sin,cos

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


def getCorners(x, y, angle, cornerNumber):
    len = Trig.distanceToPoint(0,0,getSize()/2,getSize()/2)

    #0 is upperLeft, 1 is lowerLeft, 2 is lowerRight and 3 is upperRight
    cornerNumber= 90 * (cornerNumber % 4) + 45

    cx = x + len * cos(Trig.degToRad(angle + cornerNumber))
    cy = y + len * sin(Trig.degToRad(angle + cornerNumber))

    return cx,cy


def getSides(x, y, angle, sideNumber):
    len = getSize()/2

    # 0 is up, 1 is left, 2 is down and 3 is right
    sideNumber = 90 * (sideNumber % 4)

    cx = x + len * cos(Trig.degToRad(angle + sideNumber))
    cy = y + len * sin(Trig.degToRad(angle + sideNumber))

    return cx, cy

def getBoth(gx,gy,angle,number):
    if number % 2 == 0:
        return getSides(gx,gy,angle,number)
    else:
        return getCorners(gx,gy,angle,number)


def getSize():
    #got from differential-drive
    return 0.40000000596046448