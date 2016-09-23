"""
File handles Robot movements and maths about it.
"""

import sys
import time
from math import sin, cos
from threading import Thread

import RobotState
import Trig
import path, laser
from Postman import postSpeed, getLaser
import draw

from math import pi

"""returns what direction the robot should turn,
returns -1 for clockwise, 1 for counter-clockwise"""


def turnDirection(currentAngle, goalAngle):
    currentAngle %= 360
    goalAngle %= 360
    if currentAngle < goalAngle:
        if currentAngle + 180 > goalAngle:
            return 1
        else:
            return -1
    else:
        if currentAngle - 180 < goalAngle:
            return -1
        else:
            return 1


def robotCanSee(x, y, goalx, goaly, robotDirection, laser):
    dist = Trig.distanceToPoint(x, y, goalx, goaly)

    if dist < RobotState.getActualSize() / 2: return True

    angle = Trig.angleToPoint(x, y, goalx, goaly)

    laserLength = RobotState.getLaserLength(laser, angle, robotDirection)

    return laserLength > dist


def purePursuit(x, y, goalx, goaly, angle, linearSpeed):
    goalAngle = Trig.angleToPoint(x, y, goalx, goaly)

    dist = Trig.distanceToPoint(x, y, goalx, goaly)
    vb = goalAngle - angle

    # xprim = cos(vb) * dist
    yprim = sin(vb) * dist
    # constant
    gammay = (2 * yprim) / (dist ** 2)

    angularSpeed = gammay * linearSpeed

    if abs(angularSpeed) > 2:
        # 2 times sign of angularspeed
        angularSpeed = 2 * Trig.sign(angularSpeed)
        linearSpeed = angularSpeed / gammay

    return angularSpeed, linearSpeed

def robotCanGo(x,y,gx,gy,angle,laser):
    goalAngle = Trig.angleToPoint(x,y,gx,gy)
    dist = Trig.distanceToPoint(x,y,gx,gy)

    maxCount = round(dist/RobotState.getActualSize())
    canBeCount = maxCount

    while canBeCount>0:
        nowDist = dist * float(canBeCount) / float(maxCount)
        gx = cos(goalAngle) * nowDist
        gy = sin(goalAngle) * nowDist
        if not robotCanBe(x,y,gx,gy,angle,goalAngle,laser): return False

    return True

def robotCanBe(x, y, gx, gy, robotAngle, goalAngle, laser):
    # checks the front corners

    lx, ly = RobotState.getCorners(gx, gy ,goalAngle, 0)
    rx, ry = RobotState.getCorners(gx, gy, goalAngle, 3)

    leftAngle = Trig.angleToPoint(x, y, lx, ly)
    rightAngle = Trig.angleToPoint(x, y, rx, ry)

    leftIndex = Trig.radToLaser(leftAngle, robotAngle)
    rightIndex = Trig.radToLaser(rightAngle, robotAngle)

    distance = min(Trig.distanceToPoint(x,y,lx,ly), Trig.distanceToPoint(x,y,rx,ry))

    for i in range(min(leftIndex, rightIndex), max(leftIndex,rightIndex)+1):
        if RobotState.getLaserLengthFromIndex(laser, i) < distance: return False

    return True


"""
Chooses the points the robot will aim for
"""
def safeTravel(x, y, angle, currentIndex, pathHandler, laser):

    loop = True

    # while we can  to a point, move forward
    while loop and currentIndex < pathHandler.length() - 1:
        goalx, goaly = pathHandler.position(currentIndex)
        loop = robotCanSee(x,y,goalx,goaly,angle,laser)
        if loop: currentIndex += 1

    loop = True

    if currentIndex == pathHandler.length(): currentIndex -= 1

    # while we cant go to a point, try a point closer
    while (loop and currentIndex > 0):
        goalx, goaly = pathHandler.position(currentIndex)
        dist = Trig.distanceToPoint(x, y, goalx, goaly)
        vb = Trig.angleToPoint(x, y, goalx, goaly) - angle
        # print "x %.3f, y %.3f, goalx %.3f, goaly %.3f. Angle %.3f, Dist %.3f" % (x,y,goalx,goaly,Trig.radToDeg(Trig.angleToPoint(x, y, goalx, goaly)),dist)

        radius = dist / (2 * (sin(vb)))

        loop = collisionAlongPath(x, y, goalx, goaly, angle, radius, laser)
        if loop: currentIndex -= 1

    return currentIndex


def defineGoalTreshHold(pathHandler):
    goalIndex = pathHandler.length() - 1
    index = goalIndex - 1

    gx, gy = pathHandler.position(goalIndex)
    ix, iy = pathHandler.position(index)

    while (Trig.distanceToPoint(gx, gy, ix, iy) <= 1):
        index -= 1
        ix, iy = pathHandler.position(index)

    return index + 1


def inGoal(x, y, currentIndex, threashHold, pathHandler):
    goalx, goaly = pathHandler.getLast()
    return Trig.distanceToPoint(x, y, goalx, goaly) < 1 and currentIndex > threashHold


def collisionAlongPath(x, y, goalx, goaly, robotAngle, r, laser):

    #Center of circle of turn
    cx, cy = Trig.getCenterOfTurn(r, robotAngle, x, y)
    #Angle from center to robot
    centerToRobot = Trig.angleToPoint(cx, cy, x, y)
    #Angle from center to goalPoint
    centerToGoal = Trig.angleToPoint(cx, cy, goalx, goaly)

    #The way we are going to turn
    turnDir = Trig.sign(r)

    #The difference in angle between centerToRobot and centerToGoal
    angleDiff = Trig.angleDifferenceDirection(centerToRobot, centerToGoal, turnDir)

    maxCount = round (r*angleDiff/RobotState.getActualSize())
    canBeCount = maxCount

    while (canBeCount > 0):

        checkAngle = centerToRobot + angleDiff*(float(canBeCount)/float(maxCount))

        checkRobotAngle = checkAngle + (pi / 2) * turnDir
        checkX = cx + cos(checkAngle) * abs(r)
        checkY = cy + sin(checkAngle) * abs(r)

        if not robotCanBe(x, y, checkX, checkY, robotAngle, checkRobotAngle, laser): return True

        canBeCount -=1

    return False

def mainPure(linearPreference, pathHandler, laser):

    threashHold = defineGoalTreshHold(pathHandler)
    currentIndex = 0
    goalx,goaly = 0,0
    goalAngle = 0

    start = time.time()
    sleepy = 0.1
    # so we wont sleep the first go-around
    end = start + sleepy
    x, y = RobotState.getPosition()
    okToGo = True
    startOfSimulation = time.time()

    #TODO
    # assuming straight forward rather than assume standning still
    # postSpeed(0,1)

    while not inGoal(x, y, currentIndex, threashHold, pathHandler):
        time.sleep(max(0, sleepy - (end - start)))
        x, y = RobotState.getPosition()
        pathHandler.setRobotPosition(x, y)
        angle = RobotState.getDirection()

        #LASERSTUFF
        laser.setDirection(angle)
        laser.updateLaserScan()
        laserScan = laser.getLaserScan()

        # can we see
        if okToGo:
            currentIndex = safeTravel(x, y, angle, currentIndex, pathHandler, laserScan)
            pathHandler.setCurrentIndex(currentIndex)

            goalx, goaly = pathHandler.position(currentIndex)
            goalAngle = Trig.angleToPoint(x, y, goalx, goaly)

        angleDifference = Trig.angleDifference(angle, goalAngle)

        if angleDifference < (pi / 2) and okToGo:
            ang, lin = purePursuit(x, y, goalx, goaly, angle, linearPreference)

            if angleDifference>(pi/3) and robotCanGo(x,y,goalx,goaly,angle,laserScan):
                ang = Trig.sign(ang) * 2

        else:
            ang = turnDirection(angle, goalAngle) * 2 * angleDifference / (pi / 2)
            lin = 0
            print "\tang %.3f" % Trig.radToDeg(angleDifference)
            okToGo = angleDifference < Trig.degToRad(10)

        postSpeed(ang, lin)
        end = time.time()

    endOfSimulation = time.time()
    postSpeed(0,0)

    timeTook = endOfSimulation - startOfSimulation

    print "The robot took %.3f seconds to finish the course" % (timeTook)

if __name__ == '__main__':

    pathName = sys.argv[1]

    pathHandler = path.Path(pathName)
    laserHandler = laser.Laser(0,getLaser())

    #t = Thread(target=draw.main, args=(pathHandler,laserHandler))
    #t.start()

    #1 is prefered linear speed, we prefer maxsspeed
    mainPure(1, pathHandler, laserHandler)



# This test i think is a good indicator that the laser works now,
# some times there is a diff when reading the angle upward cause the laser is not perfect and sometimes measures the
# corridor and sometimes the inner room but that is to be expected






