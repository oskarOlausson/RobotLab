"""
File that handles paths and stuff
"""

import Trig
import json

def load(fileString):
    with open(fileString ,'r') as data_file:
        data = json.load(data_file)
    return data

#returns the position of the given point
def position(index):
    return path[index]['Pose']['Position']['X'],path[index]['Pose']['Position']['Y']

def choosePoint(x,y,lookAhead,currentIndex):
    goalx,goaly=position(currentIndex)
    keepSwimming=True

    while keepSwimming:
        currentIndex+=1
        if Trig.distanceToPoint(x, y, *position(currentIndex))<lookAhead:
            goalx, goaly = position(currentIndex)
        else:
            keepSwimming=False

    return goalx,goaly


path = load("Path-to-bed.json")