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


from math import pi

"""returns what direction the robot should turn,
returns -1 for clockwise, 1 for counter-clockwise"""
def turnDirection(currentAngle, goalAngle):
    currentAngle %= 360
    goalAngle %= 360
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
    laserAngle= Trig.radToLaser(angle, robotDirection)

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

    #xprim = cos(vb) * dist
    yprim = sin(vb) * dist

    #constant
    gammay= (2 * yprim) / (dist**2)

    angularSpeed = gammay * linearSpeed

    #radius of circle
    r = 1/gammay

    if abs(angularSpeed)> 2:
        #2 times sign of angularspeed
        angularSpeed=2*Trig.sign(angularSpeed)
        linearSpeed=angularSpeed/gammay

    return angularSpeed,linearSpeed

def robotCanBe(x,y,gx,gy,angle):
    for i in range(0,8):
        cx,cy= RobotState.getBoth(gx, gy, angle, i)
        if not robotCanSee(x, y, cx, cy, RobotState.getDirection()): return False
    return True

def choosePoint(x,y,currentIndex,pathHandler):

    index = currentIndex
    gx,gy=pathHandler.position(index)

    while not robotCanSee(x,y,gx,gy,0):
        index = max(0,index-1)
        gx, gy = pathHandler.position(index)

    while robotCanSee(x,y,gx,gy,0):
        index=min(index+1,pathHandler.length()-1)
        gx,gy=pathHandler.position(index)

    if not robotCanBe(x, y, gx, gy, 0): index = max(0, index - 1)

    return index

def safeTravel(x,y,angle,linearPreference,currentIndex,pathHandler):

    index = currentIndex
    loop = True

    while(loop):
        goalx, goaly = pathHandler.position(index)
        ang, lin = purePursuit(x,y,goalx,goaly,angle,linearPreference)
        radius = lin/ang
        loop = collisionAlongPath(x,y,goalx,goaly,radius)
        if loop: index-=1

    return index

def calcTurnSpeed(angle,goalAngle,timeBetween):
    leftToTurn= Trig.angleDifference(angle, goalAngle)
    angleSpeed= leftToTurn / timeBetween
    angleSpeed=min(1,angleSpeed)
    return angleSpeed

def inGoal(x,y,pathHandler):
    return Trig.distanceToPoint(x, y, *pathHandler.getLast()) < 1

def mainPure(linearPreference, pathHandler):
    lookAhead = 1
    currentIndex = 0

    while (True):
        x, y = RobotState.getPosition()
        angle = RobotState.getDirection()

        currentIndex = choosePoint(x, y, currentIndex, pathHandler)

        currentIndex = safeTravel(x,y,angle,linearPreference,currentIndex,pathHandler)
        goalx, goaly = pathHandler.position(currentIndex)
        ang, lin = purePursuit(x,y,goalx,goaly,angle,linearPreference)

        postSpeed(ang, lin)

def collisionAlongPath(x,y,goalx,goaly,r):
    #if the path is fairly straight, just watch the end position
    if abs(r)>10: return robotCanBe(x, y, goalx, goaly, RobotState.getDirection())
    ret = False
    robotAngle = RobotState.getDirection()
    cx, cy = Trig.getCenterOfTurn(r, robotAngle, x, y)

    centerToRobot = Trig.angleToPoint(cx, cy, x, y)
    centerToGoal  = Trig.angleToPoint(cx, cy, goalx, goaly)

    angleDiff = Trig.angleDifference(centerToRobot, centerToGoal)


    turnDir = turnDirection(centerToRobot,centerToGoal)

    nrOfAngles=5
    #print "---Session---"
    for i in range(0,nrOfAngles):

        index= centerToRobot + angleDiff * (float(i+1)/nrOfAngles) * turnDir
        gx=cx+cos(index)*abs(r)
        gy=cy+sin(index)*abs(r)

        #print "\t(%.3f, %.3f)" % (gx,gy)

        angleAlongCircle = index + (pi/2)*Trig.sign(r)

        check=robotCanBe(x, y, gx, gy, angleAlongCircle)

        if not check: ret=True

    return ret

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
    x,y=0,0
    goalx,goaly=3,1
    linearPreference=1
    angle=0

    goalAngle = Trig.angleToPoint(x, y, goalx, goaly)
    linearSpeed = linearPreference

    dist = Trig.distanceToPoint(x, y, goalx, goaly)
    vb = goalAngle - angle

    yprim = sin(vb) * dist

    # constant
    gammay = (2 * yprim) / (dist ** 2)

    angularSpeed = gammay * linearSpeed

    if abs(angularSpeed)> 2:
        #2 times sign of angularspeed
        angularSpeed=2*Trig.sign(angularSpeed)
        linearSpeed=angularSpeed/gammay

    # radius of circle
    rTest = 1 / gammay

    linTest=linearSpeed

    while collisionAlongPath(x,y,goalx,goaly,rTest) and linTest>0.2:
        linTest -= 0.1
        rTest = linTest/angularSpeed

    print "linTest: %.3f, rTest: %.3f" % (linTest,rTest)

if __name__ == '__main__':
    str=sys.argv
    pathName=sys.argv[1]
    linearPreference=float(sys.argv[2])

    pathHandler = path.Path(pathName)

    if linearPreference>1 or linearPreference<=0:
        print "Unreasonable speed preference set, please set within (>0 to 1), you have set it to %.3f" % linearPreference
        exit(3)

    mainPure(linearPreference,pathHandler)










