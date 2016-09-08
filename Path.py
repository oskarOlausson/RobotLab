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

def length():
    return len(path)

path = load("Path-around-table-and-back.json")

if __name__ == '__main__':
    for i in range(0,length()):
        xx,yy = position(i)
        print "position is (%.3f, %.3f)" % (xx,yy)

