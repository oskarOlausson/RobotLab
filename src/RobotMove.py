"""
File handles Robot movements and maths about it.
"""


#forward
#forwardSpeed(M/s^2)

#stop

#turn right(target angle)
#turn left(target angle)
#turn speed(M/s^2)
import sys
import time
from math import sin,cos

import RobotState
import Trig
import path
from Postman import postSpeed,getLaser


def sign(number):
    return number>=0 - number<=0

"""returns what direction the robot should turn,
returns -1 for clockwise, 1 for counter-clockwise"""
def turnDirection(currentAngle, goalAngle):
    if currentAngle<goalAngle:
        if currentAngle+180>goalAngle:
            return 1
        else:
            return -1
    else:
        if currentAngle-180<goalAngle:
            return -1
        else:
            return 1

def robotCanSee(x,y,goalx ,goaly,robotDirection):
    angle = Trig.angleToPoint(x, y, goalx, goaly)
    laserAngle= Trig.degToLaser(angle, robotDirection)

    if laserAngle<0 or laserAngle>269:
        return True
    else:
        #print "laserAngle %d" % laserAngle
        laserLength=getLaser()['Echoes'][laserAngle]
        #print "laserLength: %.3f, length: %.3f, \nangle: %.3f, laserangle: %.3f" % (laserLength,Trig.distanceToPoint(x,y,goalx,goaly),angle,laserAngle)
        return laserLength > Trig.distanceToPoint(x, y, goalx, goaly)

def purePursuit(x,y,goalx,goaly,angle,linearPreference):

    goalAngle = Trig.angleToPoint(x, y, goalx, goaly)
    linearSpeed = linearPreference

    dist= Trig.distanceToPoint(x, y, goalx, goaly)
    vb=goalAngle-angle

    #xprim = cos(Trig.degToRad(vb)) * dist
    yprim = sin(Trig.degToRad(vb)) * dist

    #constant
    gammay= (2 * yprim) / (dist**2)

    #angularSpeed = linearSpeed * gammay
    angularSpeed = gammay * linearSpeed

    #radius of circle
    r=1/gammay

    if abs(angularSpeed)> 2:
        #2 times sign of angularspeed
        angularSpeed=2*sign(angularSpeed)
        linearSpeed=angularSpeed/gammay

    linTest=linearSpeed

    #stop us from colliding in obstacle front
    while collisionAlongPath(x,y,goalx,goaly,r,gammay*linTest) and linTest>0.1:
        linTest -= 0.1
        print "WE are GOING to CRash in front, pumps the breaks"

    if collisionAlongPath(x,y,goalx,goaly,r,gammay*linTest):
        linTest=linearSpeed

        while collisionAlongPath(x, y, goalx, goaly, r, angularSpeed) and abs(angularSpeed) > 0.1:
            angularSpeed -= 0.1*(angularSpeed/abs(angularSpeed))
            print "We are Going TO crAsh iN THE corneR!!!"

    else:
        linearSpeed=linTest

    return angularSpeed,linearSpeed

def robotCanBe(x,y,gx,gy,angle):
    for i in range(0,7):
        cx,cy= RobotState.getBoth(gx, gy, angle, i)
        if ~robotCanSee(x, y, cx, cy, RobotState.getDirection()): return False
    return True

def robotCanGo(x,y,goalx,goaly,robotDirection):
    angle= Trig.angleToPoint(x, y, goalx, goaly)
    dist= Trig.distanceToPoint(x, y, goalx, goaly)
    index= Trig.degToLaser(angle, robotDirection)
    laser = getLaser()

    if index>270 or index<0:
        print "Can't see in direction index: %d" % index
        return False

    leftIndex=index
    rightIndex=index

    #nearest point to the left
    while(leftIndex<270 and leftIndex>0 and laser['Echoes'][leftIndex]>dist):
        leftIndex+=1

    #nearest point to the right
    while(rightIndex>1 and rightIndex<270 and laser['Echoes'][rightIndex]>dist):
        rightIndex-=1

    passWidthL = laser['Echoes'][leftIndex]  * sin(Trig.degToRad(Trig.angleDifference(index, leftIndex)))
    passWidthR = laser['Echoes'][rightIndex] * sin(Trig.degToRad(Trig.angleDifference(index, rightIndex)))

    """returns whether the passing distance to the left and the passing distance to the right
    is wider than the robot"""
    return (abs(passWidthL)+abs(passWidthR)) > RobotState.getWidth()

def choosePoint(x,y,lookAhead,currentIndex,angle,pathHandler):

    keepSwimming=True
    index=currentIndex

    while keepSwimming:
        index=min(index + 1, pathHandler.length() - 1)
        goalx,goaly = pathHandler.position(index)
        distance = Trig.distanceToPoint(x, y, goalx, goaly)

        if distance<lookAhead and robotCanGo(x,y,goalx,goaly,angle):
            currentIndex=index
        else:
            keepSwimming=False

    return currentIndex

def calcTurnSpeed(angle,goalAngle,timeBetween):
    leftToTurn= Trig.angleDifference(angle, goalAngle)
    angleSpeed= Trig.degToRad(leftToTurn) / timeBetween
    angleSpeed=min(1,angleSpeed)
    return angleSpeed

def inGoal(x,y,pathHandler):
    return Trig.distanceToPoint(x, y, *pathHandler.getLast()) < 1

def mainPure(linearPreference, pathHandler):
    lookAhead = 4
    currentIndex = 0
    ready=True

    while (True):
        x, y = RobotState.getPosition()
        angle = RobotState.getDirection()

        currentIndex = choosePoint(x, y, lookAhead, currentIndex, angle,pathHandler)

        goalx, goaly = pathHandler.position(currentIndex)
        goalAngle = Trig.angleToPoint(x, y, goalx, goaly)

        if (Trig.angleDifference(angle, goalAngle)<120 and ready):
            angularSpeed, linearSpeed = purePursuit(x, y, goalx, goaly, angle,linearPreference)
        else:
            angularSpeed=turnDirection(angle,goalAngle)*2
            linearSpeed=0
            ready= Trig.angleDifference(angle, goalAngle) < 20

        postSpeed(angularSpeed, linearSpeed)

def collisionAlongPath(x,y,goalx,goaly,r,angularSpeed):
    if angularSpeed==0: return robotCanBe(x, y, goalx, goaly, RobotState.getDirection())

    cx = x + r * sin(Trig.degToRad(RobotState.getDirection() + 90 * sign(angularSpeed)))
    cy = y + r * cos(Trig.degToRad(RobotState.getDirection() + 90 * sign(angularSpeed)))

    centerToRobot = Trig.angleToPoint(cx, cy, x, y)
    centerToGoal = Trig.angleToPoint(cx, cy, goalx, goaly)

    angleDiff = Trig.angleDifference(centerToRobot, centerToGoal)
    nrOfAngles=20

    for i in range(1,nrOfAngles):
        index=centerToRobot+angleDiff*(i/nrOfAngles)*turnDirection(centerToRobot,centerToGoal)

        gx=cos(index)*r
        gy=sin(index)*r

        perpendicular=90*sign(angularSpeed)

        if ~robotCanBe(x,y,gx,gy,index+perpendicular): return False

    return True

def mainOwn():
    lookAhead = 1
    currentIndex = 0

    while (True):
        x, y = RobotState.getPosition()
        angle = RobotState.getDirection()

        currentIndex = choosePoint(x, y, lookAhead, currentIndex, angle)

        goalx, goaly = path.position(currentIndex)
        goalAngle = Trig.angleToPoint(x, y, goalx, goaly)

        angularSpeed = 2*turnDirection(angle,goalAngle)*min(1, Trig.angleDifference(angle, goalAngle) / 180)

        linearSpeed = 0.5*(1 - Trig.angleDifference(angle, goalAngle) / 90)

        postSpeed(angularSpeed, linearSpeed)
        time.sleep(1/10)

def mainRotAndDrive():
    #looks one meter ahead
    lookAhead=1
    goalx,goaly= RobotState.getPosition()
    currentIndex=0
    sp=0
    turn=0
    #estimate
    start=time.time()
    time.sleep(.1)

    while(True):
        x, y = RobotState.getPosition()
        direction = RobotState.getDirection()
        #previous position for goal
        pgoalx, pgoaly = goalx,goaly
        currentIndex=choosePoint(x,y, lookAhead, currentIndex)

        goalx, goaly = path.position(currentIndex)

        goalAngle = Trig.angleToPoint(x, y, goalx, goaly)
        end = time.time()

        if Trig.angleDifference(direction, goalAngle)>2:
            turn = turnDirection(direction, goalAngle)
            postSpeed(turn*calcTurnSpeed(direction,goalAngle,end-start),0)
        else:
            postSpeed(0,.1)

        start = time.time()
        time.sleep(.1)
def mainCheckVisability():
    x, y = 0, 0
    robotDirection = 0
    for i in range(1, 6):
        index = choosePoint(x, y, i, 0)
        gx, gy = path.position(index)
        direction = Trig.angleToPoint(x, y, gx, gy)
        laserUsed = Trig.degToLaser(direction, robotDirection)
        print "Point (%.3f, %.3f): direction:%.3f, laserUsed:%d" % (gx, gy, direction, laserUsed)
        print "\tCan see: %r, Can move: %r" % (
        robotCanSee(x, y, gx, gy, robotDirection), robotCanGo(x, y, gx, gy, robotDirection))


if __name__ == '__main__':
    # arguements in order executable path, path to the path, linearSpeedDefault
    str=sys.argv
    pathName=sys.argv[1]
    linearPreference=float(sys.argv[2])

    pathHandler = path.Path(pathName)

    if linearPreference>1 or linearPreference<=0:
        print "Unreasonable speed preference set, please set within (>0 to 1), you set it to %.3f" % linearPreference
        exit(3)

    mainPure(linearPreference,pathHandler)










