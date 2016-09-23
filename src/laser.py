"""
File that passes laserinformation to the draw-part of our debugging
"""

from Postman import getLaser

class Laser:

    def __init__(self, direction, laserScan):
        self.direction = direction
        self.laserScan = laserScan

    def updateLaserScan(self):
        self.laserScan=getLaser()

    def getLaserScan(self):
        return self.laserScan

    def setDirection(self, direction):
        self.direction = direction

    def getDirection(self):
        return self.direction




