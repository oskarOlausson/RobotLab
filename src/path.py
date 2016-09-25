"""
File that handles paths and stuff
"""

import json


class Path:
    """
        Handles the path the robot should follow
        Also passes information to the draw-object
    """

    def __init__(self, pathName):
        self.pathName = pathName
        self.path = self.load(pathName)
        self.currentIndex = 0

    def load(self, fileString):
        """
        :param fileString: the name of the file
        :return: the path
        """
        with open(fileString, 'r') as data_file:
            data = json.load(data_file)
        return data

    # returns the position of the given point
    def position(self, index):
        return self.path[index]['Pose']['Position']['X'], self.path[index]['Pose']['Position']['Y']

    def length(self):
        """
        :return: the number of poses in the path
        """
        return len(self.path)

    def getCurrentIndex(self):
        return self.currentIndex

    def setCurrentIndex(self, currentIndex):
        self.currentIndex = currentIndex

    def getLast(self):
        """
        :return: the position of the last point on the path
        """
        return self.position(self.length()-1)
