"""
File handles Robot movements and maths about it.
"""


""" + moveToCoord(XPoint, YPoint)
    while(inget är ivägen)
        1: Hitta punkten som man ska sikta på.
        2: snurra eller kör + sväng
    end

    """


#forward
#forwardSpeed(M/s^2)

#stop

#turn right(target angle)
#turn left(target angle)
#turn speed(M/s^2)

import Trig,Path

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
    laserLength=Trig.degToLaser(angle)
    print "laserLength: %.3f, length: %.3f, laserangle: %.3f" % (laserLength,Trig.distanceToPoint(x,y,goalx,goaly),angle)
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

    return goalx,goaly

