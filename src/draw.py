#!/usr/bin/python
# -*- coding: utf-8 -*-

"""
ZetCode Tkinter tutorial

This script draws lines on
the Canvas.

Author: Jan Bodnar
Last modified: November 2015
Website: www.zetcode.com
"""

from Tkinter import Tk, Canvas, Frame, BOTH
import RobotState, Postman, time
from math import pi, cos ,sin
import Trig

class Example(Frame):

    x,y = 0, 0
    gx, gy = 0,0

    prevx, prevy = 300,300
    prevgx, prevgy = prevx, prevy
    canvas = None
    pathHandler = None

    def __init__(self, parent, pathHandler, laser):
        Frame.__init__(self, parent)
        self.pathHandler = pathHandler
        self.laser = laser
        self.parent = parent
        self.initUI(laser,pathHandler)

    def initUI(self,laser,pathHandler):
        self.canvas = Canvas(self)
        self.parent.title("Lines")
        self.pack(fill=BOTH, expand=1)

        self.canvas.pack(fill=BOTH, expand=1)
        px,py=0,0
        self.after(1000, self.draw)

        for x in range(0,500,30):
            self.canvas.create_line(x,0,x,500, fill="blue")

        for y in range(0,500,30):
            self.canvas.create_line(0,y,500,y, fill="blue")

        px,py=300,300
        for i in range(0,pathHandler.length()):
            x,y = pathHandler.position(i)
            x=-x*20+300
            y=-y*20+300
            self.canvas.create_line(py,px,y,x, fill="red")

            if i==986 or i==1048:
                self.canvas.create_oval(y-2,x-2,y+2,x+2, fill="black")

            px,py=x,y

    def draw(self):
        x,y = self.pathHandler.getRobotPosition()
        index = self.pathHandler.getCurrentIndex()
        gx, gy = self.pathHandler.position(index)

        self.gx = -gx*20 + 300
        self.gy = -gy*20 + 300

        self.x  = - x*20 + 300
        self.y  = - y*20 + 300

        self.canvas.create_line(self.prevy, self.prevx, self.y, self.x, width = 2)

        self.canvas.create_line(self.gy-1, self.gx-1, self.gy+1, self.gx+1, fill="purple", width = 2)
        self.canvas.create_line(self.gy+1, self.gx-1, self.gy-1, self.gx+1, fill="purple", width = 2)

        #self.canvas.create_line(self.y,self.x, self.gy, self.gx, fill="pink", width = 3)

        self.prevx=self.x
        self.prevy=self.y
        self.prevgx = self.gx
        self.prevgy = self.gy

        robotDirection = self.laser.getDirection()
        laserScan=self.laser.getLaserScan()

        for i in range(0,360):
            rad = Trig.degToRad(i)
            index = int(Trig.radToLaser(rad,robotDirection))
            if (index>0 and index<270):
                length = laserScan['Echoes'][index]
                gx= x + cos(rad)*length
                gy= y + sin(rad)*length

                gx= -gx*20 + 300
                gy= -gy*20 + 300

            self.canvas.create_line(gy,gx,gy+2,gx+2)


        # arrange for the next frame to draw in 1/2 seconds
        self.after(250, self.draw)





def main(pathHandler,laser):
    root = Tk()
    ex = Example(root,pathHandler,laser)
    root.geometry("500x500+300+300")
    root.mainloop()
    return ex

