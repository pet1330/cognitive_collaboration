#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Cognative Collaboration Study application

:Author: Peter Lightbody <plightbody@lincoln.ac.uk>
:Organization: University of Lincoln
:Date: 10 March 2017
:Version: 0.0.1
:Status: Development
:Copyright: MIT

"""

from random import choice, uniform
import math

import matplotlib.pyplot as plt
import numpy as np

def main():
    we = World_env()
    we.load_config()
    we.seed()
    # print we.object_list[0].distance_to_line( (0.05, 75 ), (0.05, 75 ) )
    print we.object_list[0].dist_to_segment(2, 2, 4, 4)

class World_env():

    def __init__(self):
        self.left_side_width = 2
        self.right_side_width = 2
        self.grid_width = 5
        self.grid_height = 5
        self.object_list = list()


    def __str__(self):
        to_return = ((" {} " * self.grid_width) + "\n") * self.grid_height
        a = [i.box_id for i in self.object_list]
        print a
        print to_return
        to_return.format(*a)
        return to_return

    def load_config(self):
        pass

    def seed(self):
        for x in xrange(self.grid_width*self.grid_height):
            b = box()
            b.box_id = x
            b.tracking_ids_back = 0
            b.tracking_ids_back = 1
            b.tracking_ids_back = 2
            b.line_direction = choice( [ 45, 75, 90, 95, 120 ] )
            b.line_length = choice( [0.02, 0.04, 0.06, 0.08 ] )
            b.grid_position = (x%5, x/5)
            self.object_list.append(b)

    def sort_by_line_length(self):
        return sorted(self.object_list, key=lambda x: x.line_length)

    def sort_by_line_direction(self):
        return sorted(self.object_list, key=lambda x: x.line_direction)

class box():

    def __init__(self):
        self.box_id = 0
        self.tracking_ids_back = -1
        self.tracking_ids_right = -1
        self.tracking_ids_left = -1
        self.line_length = -1
        self.line_direction = -1
        self.grid_position = (-1, -1)
        selected = False
        on_grid = True
        on_left_side_panel = not on_grid
        on_right_side_panel = not on_grid



    def dist_to_segment(self, ax, ay, bx, by):
        """
        Computes the minimum distance between a point (cx, cy) and a line segment with endpoints (ax, ay) and (bx, by).
        :param ax: endpoint 1, x-coordinate
        :param ay: endpoint 1, y-coordinate
        :param bx: endpoint 2, x-coordinate
        :param by: endpoint 2, y-coordinate
        :return: minimum distance between point and line segment
        """
        cx = self.line_length
        cy = self.line_direction

        # avoid divide by zero error
        a = max(by - ay, 0.00001)
        b = max(ax - bx, 0.00001)
        # compute the perpendicular distance to the theoretical infinite line
        dl = abs(a * cx + b * cy - b * ay - a * ax) / math.sqrt(a**2 + b**2)
        # compute the intersection point
        x = ((a / b) * ax + ay + (b / a) * cx - cy) / ((b / a) + (a / b))
        y = -1 * (a / b) * (x - ax) + ay
        # decide if the intersection point falls on the line segment
        if (ax <= x <= bx or bx <= x <= ax) and (ay <= y <= by or by <= y <= ay):
            return dl
        else:
            # if it does not, then return the minimum distance to the segment endpoints
            return min(math.sqrt((ax - cx)**2 + (ay - cy)**2), math.sqrt((bx - cx)**2 + (by - cy)**2))

    def distance_to_line(self, p1, p2):
        x_diff = p2[0] - p1[0]
        y_diff = p2[1] - p1[1]
        num = abs(y_diff*self.line_direction - x_diff*self.line_length + p2[0]*p1[1] - p2[1]*p1[0])
        den = math.sqrt(math.pow(y_diff, 2) + math.pow(x_diff,2))
        print y_diff, x_diff
        print den
        return num / den

    def __str__(self):
        return """
                Box ID: %i   Position: (%i, %i)
                -------------------------------
                Tracking IDs: [%i %i %i]
                Line Length: %f
                Line Direction: %s
                ===============================""" % (
                        self.box_id,
                        self.grid_position[0],
                        self.grid_position[1],
                        self.tracking_ids_back, 
                        self.tracking_ids_right, 
                        self.tracking_ids_left,
                        self.line_length,
                        self.line_direction,
                    )



class gui():
    def __init__(self):
        plt.plot([1,2,3,4], [1,4,9,16], 'ro')
        plt.axis([0, 6, 0, 20])
        plt.show()




# if __name__ == "__main__":
#     main()
#     g = gui()


fig = plt.figure()
plot = fig.add_subplot(111)

# create some curves
for i in range(4):
    plot.plot(
        [i*1,i*2,i*3,i*4],
        gid=i)

def on_plot_hover(event):
    for curve in plot.get_lines():
        if curve.contains(event)[0]:
            print "over %s" % curve.get_gid()

fig.canvas.mpl_connect('motion_notify_event', on_plot_hover)           
plt.show()