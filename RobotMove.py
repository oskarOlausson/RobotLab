"""
File handles Robot movements and maths about it.
"""


#forward
#forwardSpeed(M/s^2)

#stop

#turn right(target angle)
#turn left(target angle)
#turn speed(M/s^2)
import time
import Trig,Path,RobotState
from math import sin,cos,sqrt
from Postman import postSpeed,getLaser

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

def robotCanSee(x,y,goalx ,goaly):
    angle = Trig.angleToPoint(x,y,goalx,goaly)
    laserAngle=Trig.degToLaser(angle)

    if laserAngle<0 or laserAngle>269:
        return True
    else:
        #print "laserAngle %d" % laserAngle
        laserLength=getLaser()['Echoes'][laserAngle]
        #print "laserLength: %.3f, length: %.3f, \nangle: %.3f, laserangle: %.3f" % (laserLength,Trig.distanceToPoint(x,y,goalx,goaly),angle,laserAngle)
        return laserLength>Trig.distanceToPoint(x,y,goalx,goaly)

def purePursuit(x,y,goalx,goaly,lookAhead):
    linearSpeed=0.4

    gamma=(2*(goaly-y))/(lookAhead**2)
    angularSpeed=linearSpeed*gamma
    return angularSpeed,linearSpeed

def robotCanGo(x,y,goalx,goaly):
    angle=Trig.angleToPoint(0,0,goalx,goaly)
    dist=Trig.distanceToPoint(0,0,goalx,goaly)
    index=Trig.degToLaser(angle)
    laser = getLaser()

    leftIndex=index
    rightIndex=index

    while(leftIndex<Path.length()-1 and laser['Echoes'][leftIndex]>dist):
        leftIndex+=1

    while(rightIndex>1 and laser['Echoes'][rightIndex]>dist):
        rightIndex-=1

    lx,ly=Path.position(leftIndex)
    rx,ry=Path.position(rightIndex)
    rlAngle=Trig.angleToPoint(lx,ly,rx,ry)

    dist = Trig.distanceToPoint(lx,ly,rx,ry) * cos(Trig.degToRad(Trig.angleDifference(angle-90,rlAngle)))

    return dist>RobotState.getWidth()

def choosePoint(x,y,lookAhead,currentIndex):
    goalx,goaly=Path.position(currentIndex)
    keepSwimming=True
    index=currentIndex

    while keepSwimming:
        index=min(index+1,Path.length()-1)
        goalx,goaly = Path.position(index)
        distance = Trig.distanceToPoint(x, y, goalx, goaly)

        if distance<lookAhead and robotCanSee(x,y,goalx,goaly):
            currentIndex=index
            print "can see point %d, %.3f, %.3f" % (index,goalx,goaly)
        else:
            keepSwimming=False

    return currentIndex

def calcTurnSpeed(angle,goalAngle,timeBetween):
    leftToTurn=abs(Trig.angleDifference(angle,goalAngle))
    angleSpeed=Trig.degToRad(leftToTurn)/timeBetween
    angleSpeed=min(1,angleSpeed)
    return angleSpeed

if __name__ == '__main__':
    currentIndex=0
    lookAhead = 2
    while(True):
        x, y = RobotState.getPosition()
        currentIndex=choosePoint(x,y,lookAhead,currentIndex)
        goalx,goaly=Path.position(currentIndex)
        postSpeed(*purePursuit(x,y,goalx,goaly,lookAhead))
        print "goalx,goaly: (%.3f, %.3f)" % (goalx,goaly)
        time.sleep(.1)

def notMain():
    #looks one meter ahead
    lookAhead=1
    goalx,goaly=RobotState.getPosition()
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
        print "currentIndex: %d" % currentIndex
        goalx, goaly = Path.position(currentIndex)

        goalAngle = Trig.angleToPoint(x, y, goalx, goaly)
        end = time.time()

        if (Trig.angleDifference(direction,goalAngle)>1):
            turn = turnDirection(direction, goalAngle)
            postSpeed(turn*calcTurnSpeed(direction,goalAngle,end-start),0)
        else:
            postSpeed(0,.1)

        start = time.time()
        time.sleep(.1)




