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


class Example(Frame):

    x,y = 5, 0
    gx, gy = 0,0

    prevx, prevy = x,y
    prevgx, prevgy = gx, gy
    canvas = None
    pathHandler = None

    def __init__(self, parent, pathHandler):
        Frame.__init__(self, parent)
        self.pathHandler = pathHandler

        self.parent = parent
        self.initUI()

    def initUI(self):
        self.canvas = Canvas(self)
        self.parent.title("Lines")
        self.pack(fill=BOTH, expand=1)

        self.canvas.pack(fill=BOTH, expand=1)
        px,py=0,0
        self.after(1000, self.draw)

    def draw(self):
        x,y = self.pathHandler.getRobotPosition()
        index = self.pathHandler.getCurrentIndex()
        gx, gy = self.pathHandler.position(index)

        self.gx = gx*15 + 200
        self.gy = gy*15 +200

        self.x=-x*15+200
        self.y=-y*15+200


        self.canvas.create_line(self.prevy, self.prevx, self.y, self.x)
        self.canvas.create_line(self.prevgy, self.prevgx, self.gy, self.gx)

        self.prevx=self.x
        self.prevy=self.y
        self.prevgx = self.gx
        self.prevgy = self.gy

        # arrange for the next frame to draw in 1 seconds
        self.after(1000, self.draw)




def main(pathHandler):
    root = Tk()
    ex = Example(root,pathHandler)
    root.geometry("400x250+300+300")
    root.mainloop()
    return ex


if __name__ == '__main__':
    main()