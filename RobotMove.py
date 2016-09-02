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
from Postman import postSpeed

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

def robotCanSee(goalx ,goaly):
    x,y=RobotState.getPosition()
    angle = Trig.angleToPoint(x,y,goalx,goaly)
    laserLength=Trig.degToLaser(angle)
    #print "laserLength: %.3f, length: %.3f, laserangle: %.3f" % (laserLength,Trig.distanceToPoint(x,y,goalx,goaly),angle)
    return laserLength>Trig.distanceToPoint(x,y,goalx,goaly)

def choosePoint(x,y,lookAhead,currentIndex):
    goalx,goaly=Path.position(currentIndex)
    keepSwimming=True

    while keepSwimming:
        currentIndex+=1
        if Trig.distanceToPoint(x, y, *Path.position(currentIndex))<lookAhead:
            goalx, goaly = Path.position(currentIndex)
        else:
            keepSwimming=False

    return currentIndex

if __name__ == '__main__':
    path = Path.load('Path-to-bed.json')

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
        pgoalx, pgoaly = goalx, goaly
        currentIndex=choosePoint(x, y, lookAhead, currentIndex)
        goalx, goaly = Path.position(currentIndex)

        goalAngle = Trig.angleToPoint(x, y, goalx, goaly)

        if (not robotCanSee(goalx,goaly)):
            if (pgoalx != goalx or pgoaly != goaly):
                print "Could not see the point (of this):   %.3f, %.3f" % (goalx,goaly)
        else:
            if (pgoalx!=goalx or pgoaly!=goaly):
                print "Hey! I found a point, lets go there: %.3f, %.3f" % (goalx, goaly)

            if (Trig.angleDifference(direction,goalAngle)>5):
                turn = turnDirection(direction, goalAngle)
                postSpeed(turn*0.5,0)
            else:
                postSpeed(0,1)

            time.sleep(.1)



