"""
File that handles paths and stuff
"""

import Trig
import json


class Path:
    def __init__(self, pathName):
        self.pathName = pathName
        self.path = self.load(pathName)

    def load(self, fileString):
        with open(fileString, 'r') as data_file:
            data = json.load(data_file)
        return data

    # returns the position of the given point
    def position(self, index):
        return self.path[index]['Pose']['Position']['X'], self.path[index]['Pose']['Position']['Y']

    def length(self):
        return len(self.path)
