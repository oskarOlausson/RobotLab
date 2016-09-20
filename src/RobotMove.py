"""
File handles Robot movements and maths about it.
"""

import sys
import time
from math import sin,cos
from threading import Thread

import RobotState
import Trig
import path
from Postman import postSpeed,getLaser
import draw



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

def robotCanSee(x, y, goalx, goaly, robotDirection, laser):
    dist = Trig.distanceToPoint(x, y, goalx, goaly)

    if dist<RobotState.getActualSize()/2: return True

    angle = Trig.angleToPoint(x, y, goalx, goaly)
    laserAngle= Trig.radToLaser(angle, robotDirection)

    laserLength=RobotState.getLaserIndex(laser,laserAngle)
    return laserLength > dist

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

    if abs(angularSpeed)> 2:
        #2 times sign of angularspeed
        angularSpeed=2*Trig.sign(angularSpeed)
        linearSpeed=angularSpeed/gammay

    return angularSpeed,linearSpeed

def robotCanBe(x, y, gx, gy, angle, goalAngle, laser):
    #checks the front corners
    for i in [0,3]:
        #Dont check the backcorners if we are to close, will get false readings
        cx,cy= RobotState.getCorners(gx, gy, goalAngle, i)
        if not robotCanSee(x, y, cx, cy, angle, laser): return False
    return True

def choosePoint(x, y, angle, currentIndex,lookAhead, pathHandler, laser):

    index = currentIndex
    gx,gy = pathHandler.position(index)


    while not robotCanBe(x, y, gx, gy, angle, angle, laser) and index>0:
        index = max(0,index-1)
        gx, gy = pathHandler.position(index)
        print "cant see point (%.3f, %.3f)" % (gx,gy)

    while robotCanBe(x, y, gx, gy, angle, angle, laser) and Trig.distanceToPoint(x,y,gx,gy)<=lookAhead:
        index=min(index+1,pathHandler.length()-1)
        gx,gy=pathHandler.position(index)

    if not robotCanBe(x, y, gx, gy, angle, angle, laser): index = max(0, index - 1)

    return index

def safeTravel(x, y, angle, currentIndex, pathHandler, laser):

    loop = True
    goalx, goaly = pathHandler.position(currentIndex)
    dist = Trig.distanceToPoint(x, y, goalx, goaly)

    #while we can move to a point, move forward
    while loop and currentIndex <= pathHandler.length()-1:

        goalx, goaly = pathHandler.position(currentIndex)
        dist = Trig.distanceToPoint(x, y, goalx, goaly)
        vb = Trig.angleToPoint(x, y, goalx, goaly) - angle

        radius = dist / (2 * (sin(vb)))
        #loop == false when we can not travel to point
        loop = not collisionAlongPath(x, y, goalx, goaly, angle, radius, laser)
        if loop: currentIndex += 1

    loop = True

    if currentIndex==pathHandler.length(): currentIndex-=1

    #while we cant go to a point, try a point closer
    while(loop and currentIndex>=0):
        goalx, goaly = pathHandler.position(currentIndex)
        dist = Trig.distanceToPoint(x, y, goalx, goaly)
        vb = Trig.angleToPoint(x, y, goalx, goaly) - angle
        #print "x %.3f, y %.3f, goalx %.3f, goaly %.3f. Angle %.3f, Dist %.3f" % (x,y,goalx,goaly,Trig.radToDeg(Trig.angleToPoint(x, y, goalx, goaly)),dist)


        radius = dist / (2 * (sin(vb)))

        loop = collisionAlongPath(x, y, goalx, goaly, angle, radius, laser) and Trig.distanceToPoint(x,y,goalx,goaly)>0.5
        if loop: currentIndex -= 1

    return currentIndex


def defineGoalTreshHold(pathHandler):
    goalIndex = pathHandler.length()-1
    index = goalIndex-1

    gx,gy=pathHandler.position(goalIndex)
    ix,iy=pathHandler.position(index)

    while(Trig.distanceToPoint(gx,gy,ix,iy)<=1):
        index-=1
        ix, iy = pathHandler.position(index)

    return index+1

def calcTurnSpeed(angle,goalAngle,timeBetween):
    leftToTurn= Trig.angleDifference(angle, goalAngle)
    angleSpeed= leftToTurn / timeBetween
    angleSpeed=min(1,angleSpeed)
    return angleSpeed

def inGoal(x,y,currentIndex,threashHold,pathHandler):
    goalx,goaly = pathHandler.getLast()
    return Trig.distanceToPoint(x, y, goalx,goaly) < 1 and currentIndex>threashHold

def collisionAlongPath(x, y, goalx, goaly, robotAngle, r, laser):

    cx, cy = Trig.getCenterOfTurn(r, robotAngle, x, y)

    centerToRobot = Trig.angleToPoint(cx, cy, x, y)
    centerToGoal  = Trig.angleToPoint(cx, cy, goalx, goaly)

    turnDir = Trig.sign(r)
    angleDiff = Trig.angleDifferenceDirection(centerToRobot, centerToGoal,turnDir)

    halfPoint = centerToRobot + angleDiff / 2
    midX = cx + cos(halfPoint) * abs(r)
    midY = cy + sin(halfPoint) * abs(r)

    goalAngle = halfPoint + (pi / 2) * turnDir
    canBeMid = robotCanBe(x,y,midX,midY,robotAngle,goalAngle,laser)

    return not canBeMid or not robotCanBe(x,y, goalx, goaly, robotAngle, goalAngle, laser)

def mainPure(linearPreference, pathHandler):
    threashHold=defineGoalTreshHold(pathHandler)
    currentIndex = 0
    start = time.time()
    sleepy = 0.1
    #so we wont sleep the first go-around
    end = start + sleepy
    x, y = RobotState.getPosition()
    okToGo = True
    startOfSimulation = time.time()
    lookAhead = 1.5
    #assuming straight forward rather than assume standning still
    #postSpeed(0,1)

    while not inGoal(x,y,currentIndex,threashHold,pathHandler):
        time.sleep(max(0,sleepy-(end-start)))
        x, y = RobotState.getPosition()
        pathHandler.setRobotPosition(x,y)
        angle = RobotState.getDirection()
        laser = getLaser()
        #can we see
        currentIndex = safeTravel(x,y,angle,currentIndex,pathHandler,laser)
        pathHandler.setCurrentIndex(currentIndex)

        goalx, goaly = pathHandler.position(currentIndex)
        goalAngle = Trig.angleToPoint(x,y,goalx,goaly)
        angleDifference = Trig.angleDifference(angle,goalAngle)

        if angleDifference<(pi/2) and okToGo:
            goalx, goaly = pathHandler.position(currentIndex)
            ang, lin = purePursuit(x, y, goalx, goaly, angle, linearPreference)
        else:
            ang = turnDirection(angle,goalAngle)*2 * angleDifference/(pi/2)
            lin = 0
            okToGo = angleDifference < Trig.degToRad(20)

        postSpeed(ang, lin)
        end = time.time()

    endOfSimulation = time.time()

    timeTook = endOfSimulation - startOfSimulation

    print "The robot took %.3f seconds to finish the course" % (timeTook)

def mainOwn(pathHandler):
    currentIndex = 0
    lookAhead=1

    while (True):
        x, y = RobotState.getPosition()
        angle = RobotState.getDirection()

        currentIndex = choosePoint(x,y, currentIndex, angle, lookAhead, pathHandler)

        goalx, goaly = pathHandler.position(currentIndex)
        goalAngle = Trig.angleToPoint(x, y, goalx, goaly)

        angularSpeed = 2*turnDirection(angle,goalAngle)*min(1, Trig.angleDifference(angle, goalAngle) / 180)

        linearSpeed = 0.5*(1 - Trig.angleDifference(angle, goalAngle) / 90)

        postSpeed(angularSpeed, linearSpeed)
        time.sleep(1/10)

def extensiveCanSeeTest():
    x,y = RobotState.getPosition()
    rDir = RobotState.getDirection()
    laser = getLaser()

    for len in range(1,10):
        len2 = len/2
        #for dir in range(-135,135,15):
        dir = 0
        angle = Trig.degToRad(dir)
        check = robotCanSee(x, y, cos(angle) * len2, sin(angle) * len2, rDir, laser)
        if check: print "Robot can see point goalx: %.3f, goaly %.3f," % (x+cos(angle)*len2,y+sin(angle)*len2)
        else: print "\t!Robot can not see point goalx: %.3f, goaly %.3f," % (x+cos(angle)*len2,y+sin(angle)*len2)

if __name__ == '__main__':
    pathName=sys.argv[1]
    linearPreference=float(sys.argv[2])

    pathHandler = path.Path(pathName)

    if linearPreference>1 or linearPreference<=0:
        print "Unreasonable speed preference set, please set within (>0 to 1), you have set it to %.3f" % linearPreference
        exit(3)

    t = Thread(target=draw.main, args=(pathHandler,))
    t.start()
    mainPure(linearPreference,pathHandler)



#This test i think is a good indicator that the laser works now,
#some times there is a diff when reading the angle upward cause the laser is not perfect and sometimes measures the
#corridor and sometimes the inner room but that is to be expected






