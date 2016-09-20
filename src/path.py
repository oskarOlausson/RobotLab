"""
File that handles paths and stuff
"""

import json


class Path:

    def __init__(self, pathName):
        self.pathName = pathName
        self.path = self.load(pathName)
        self.currentIndex = 0
        self.x, self.y = 0,0

    def load(self, fileString):
        with open(fileString, 'r') as data_file:
            data = json.load(data_file)
        return data

    # returns the position of the given point
    def position(self, index):
        return self.path[index]['Pose']['Position']['X'], self.path[index]['Pose']['Position']['Y']

    def length(self):
        return len(self.path)

    def getCurrentIndex(self):
        return self.currentIndex

    def setCurrentIndex(self, currentIndex):
        self.currentIndex = currentIndex

    def getRobotPosition(self):
        return self.x, self.y

    def setRobotPosition(self,x,y):
        self.x=x
        self.y=y

    def getLast(self):
        return self.position(self.length()-1)


if __name__ == '__main__':
    p = Path("Path-around-table.json")
    x,y = p.position(62)
    print "x %.3f, y %.3f" % (x,y)