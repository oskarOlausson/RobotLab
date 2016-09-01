
import json, RobotControl
from math import sin, cos, pi, atan2, sqrt


def load(fileString):
    with open(fileString ,'r') as data_file:
        data = json.load(data_file)
    return data

#Converts from radians to degrees
def radToDeg(rad):
    return rad * ( 180 /pi)

#converts drom degrees to radians
def degToRad(deg):
    return deg * ( pi /180)

#takes an angle (in degrees) and returns the laser that points in that direction
#...hopefully (in the test it seems to work
def degToLaser(deg):
    laserData = RobotControl.getLaser()
    unitVector = RobotControl.getBearing()
    ux=unitVector['X']
    uy=unitVector['Y']
    robotAngle = radToDeg(atan2(uy ,ux))
    # print "current angle being investigatet is %.3f" % deg
    #print "robotAngle is %.3f" % robotAngle
    # there is a 45 degrees difference between the laser and the degrees
    laserAngle = int( deg + 45 -robotAngle)
    #print "robotlaser chosen is %.3f" % laserAngle
    return laserData['Echoes'][laserAngle]

def distanceToPoint(x,y,goalx,goaly):
    return sqrt((goalx-x) ** 2 + ( goaly-y) ** 2 )

def straightLength(length,direction):
    return cos(degToRad(direction))*length

def angleToPoint(x,y,goalx,goaly):
    return radToDeg(atan2(goaly-y, goalx-x))

#looks for a point and sees if the laser can look that far
#if it can't, there is a wall in the way
#returns a bool, canSee
def robotCanSee(x,y,goalx ,goaly):
    angle = angleToPoint(x,y,goalx,goaly)
    laserLength=degToLaser(angle)
    print "laserLength: %.3f, length: %.3f, laserangle: %.3f" % (laserLength,distanceToPoint(x,y,goalx,goaly),angle)
    return laserLength>distanceToPoint(x,y,goalx,goaly)



if __name__ == '__main__':
    data = load("Path-to-bed.json")

    print 'First point of x-data is %.3f' % (data[0]['Pose']['Position']['X'])
    print 'First point of y-data is %.3f' % (data[0]['Pose']['Position']['Y'])
    print 'Last point of x-data is %.3f' % (data[len(data)-1][ 'Pose']['Position']['X'])
    print 'Last point of y-data is %.3f' % (data[len(data)-1][ 'Pose']['Position']['Y'])

    x,y = RobotControl.getPosition()
    step = 10

    #creates an array that will fit all readings
    canSeeList = [False] * int (len(data)/step)

    print "My position is point(%.3f,%.3f)" % (x,y)

    for i in range(0, (int(len(data)/step)) - 1):
        goalx = data[i*step]['Pose']['Position']['X']
        goaly = data[i*step]['Pose']['Position']['Y']

        canSeeList[i] = robotCanSee(x,y,goalx,goaly)


        if canSeeList[i]:
            print ""
            #print "The robot can see point(%.3f, %.3f), distance to point is: %.3f"   % (goalx,goaly,distanceToPoint(x,y,goalx,goaly))
        else:
            print "The robot can't see point(%.3f, %.3f), distance to point is: %.3f " % (goalx,goaly,distanceToPoint(x,y,goalx,goaly))




