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
        return False
    else:
        #print "laserAngle %d" % laserAngle
        laserLength=getLaser()['Echoes'][laserAngle]
        #print "laserLength: %.3f, length: %.3f, \nangle: %.3f, laserangle: %.3f" % (laserLength,Trig.distanceToPoint(x,y,goalx,goaly),angle,laserAngle)
        return laserLength>Trig.distanceToPoint(x,y,goalx,goaly)

def robotCanGo(x,y,goalx,goaly):
    angle=Trig.angleToPoint(0,0,goalx,goaly)
    angleLeft=angle+90
    #robots width/2 according to the physics
    robotHalfWidth=.2
    sidex=cos(Trig.degToRad(angleLeft))*robotHalfWidth
    sidey=sin(Trig.degToRad(angleLeft))*robotHalfWidth
    #checks in front, to the left and right
    return robotCanSee(x,y,goalx,goaly) and robotCanSee(x,y,goalx+sidex,goaly+sidey) and robotCanSee(x,y,goalx-sidex,goaly-sidey)


def choosePoint(x,y,lookAhead,currentIndex):
    goalx,goaly=Path.position(currentIndex)
    keepSwimming=True
    index=currentIndex

    while keepSwimming:
        index=min(index+1,Path.length()-1)
        goalx,goaly = Path.position(index)
        distance = Trig.distanceToPoint(x, y, goalx, goaly)

        if robotCanGo(x,y,goalx,goaly):
            currentIndex=index
            print "can see point %d, %.3f, %.3f" % (index,goalx,goaly)
        else:
            keepSwimming=False

    return currentIndex

if __name__ == '__main__':
    #looks one meter ahead
    lookAhead=1
    goalx,goaly=RobotState.getPosition()
    currentIndex=0
    sp=0
    turn=0
    while(True):
        x, y = RobotState.getPosition()
        direction = RobotState.getDirection()
        #previous position for goal
        pgoalx, pgoaly = goalx,goaly
        currentIndex=choosePoint(x,y, lookAhead, currentIndex)
        print "currentIndex: %d" % currentIndex
        goalx, goaly = Path.position(currentIndex)

        goalAngle = Trig.angleToPoint(x, y, goalx, goaly)

        if (Trig.angleDifference(direction,goalAngle)>10):
            turn = turnDirection(direction, goalAngle)
            postSpeed(turn*0.25,0)
        else:
            postSpeed(0,.1)

            time.sleep(.1)



