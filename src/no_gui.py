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

import random
import sys
import time
from math import sqrt
import actionlib
import baxter_interface
from mary_tts.msg import maryttsAction, maryttsGoal
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt
import numpy as np
import rospkg
import rospy
import yaml
from PyQt4.QtGui import QApplication, QDialog, QGridLayout, QHBoxLayout, QListWidget, QPushButton, QVBoxLayout
from PyQt4 import QtCore

import rospy
from std_msgs.msg import String


class Gui(QDialog):
    def __init__(self, object_list):
        app = QApplication([0])
        super(Gui, self).__init__()
        self.object_list = object_list

    def gui_setup(self):
        self.figure = plt.figure()
        self.canvas = FigureCanvas(self.figure)
        self.plot()
        self.plot()  # double plot centres the graph (not sure why this doesn't happen first time)

        # set the layout
        self.baseLayout = QVBoxLayout()
        self.sceneLayout = QHBoxLayout()
        self.listLayout = QHBoxLayout()
        self.gridLayout = QGridLayout()
        for button in self.object_list:
            self.gridLayout.addWidget(button, button.grid_position_x, button.grid_position_y)
            button.clicked.connect(self.button_click)

        cat_a_list = QListWidget()
        cat_b_list = QListWidget()

        for i in sorted(self.object_list, key=lambda x: x.difficulty):
            if i.difficulty > 0.00001:
                if i.category == "A" and i.difficulty > 0.0:
                    cat_a_list.addItem('%s (%0.4f)' % (i.box_id, i.difficulty))
                else:
                    cat_b_list.addItem('%s (%0.4f)' % (i.box_id, i.difficulty))
        # a.item(2).setSelected(True)
        # b.item(5).setSelected(True)

        self.listLayout.addWidget(cat_a_list)
        self.listLayout.addWidget(cat_b_list)
        self.sceneLayout.addWidget(self.canvas)
        self.sceneLayout.addLayout(self.gridLayout)
        self.baseLayout.addLayout(self.sceneLayout)
        self.baseLayout.addLayout(self.listLayout)
        self.setLayout(self.baseLayout)


class Box(object):

    def __init__(self, parent=None):
        super(Box, self).__init__()
        self.box_id = None
        self.tracking_ids_back = -1
        self.tracking_ids_right = -1
        self.tracking_ids_left = -1
        self.line_length = -1
        self.line_direction = -1
        self.grid_position = (-1, -1)
        self.category = ""
        self.selected = False
        self.on_grid = True
        self.on_left_side_panel = not self.on_grid
        self.on_right_side_panel = not self.on_grid

    def dist_to_segment(self, ax, ay, bx, by):
        cx = self.line_length
        cy = self.line_direction

        # avoid divide by zero error
        a = max(by - ay, 0.0000000000000000000000001)
        b = max(ax - bx, 0.0000000000000000000000001)
        # compute the perpendicular distance to the theoretical infinite line
        dl = abs(a * cx + b * cy - b * ay - a * ax) / sqrt(a**2 + b**2)
        # compute the intersection point
        x = ((a / b) * ax + ay + (b / a) * cx - cy) / ((b / a) + (a / b))
        y = -1 * (a / b) * (x - ax) + ay
        # decide if the intersection point falls on the line segment
        if (ax <= x <= bx or bx <= x <= ax) and (ay <= y <= by or by <= y <= ay):
            return dl
        else:
            # if it does not, then return the minimum distance to the segment endpoints
            return min(sqrt((ax - cx)**2 + (ay - cy)**2), sqrt((bx - cx)**2 + (by - cy)**2))


class Controller(object):
    def __init__(self):
        super(Controller, self).__init__()
        

    def load_config(self):
        with open(rospkg.RosPack().get_path('cognative_collaboration') + '/config/config.yaml', 'r') as stream:
            params = yaml.load(stream)
            # ============== Grid Settings ======================
            self.left_side_width = params['grid']['sides']['left']
            self.right_side_width = params['grid']['sides']['right']
            self.grid_width = params['grid']['size']['width']
            self.grid_height = params['grid']['size']['height']
            self.grid_cell_width = params['grid']['cell_size']['width']
            self.grid_cell_height = params['grid']['cell_size']['height']
            self.grid_zero_zero_position = params['grid']['offset']
            # ============== Box Settings ======================
            self.object_list = list()
            for item in params['object_list']:
                for key, body in item.items():
                    b = Box()
                    b.box_id = key
                    b.tracking_ids_back, b.tracking_ids_back, b.tracking_ids_back = body['tracking_ids']
                    b.line_length = body['line_length']
                    b.line_direction = body['line_direction']
                    b.grid_position_x = body['grid_position'][0]
                    b.grid_position_y = body['grid_position'][1]
                    b.world_position_x = self.grid_zero_zero_position[0] + (self.grid_cell_height * b.grid_position_x)
                    b.world_position_y = self.grid_zero_zero_position[1] + (self.grid_cell_width * b.grid_position_y)
                    b.world_position_z = self.grid_zero_zero_position[2]
                    self.object_list.append(b)
            # ============== Box calculations ======================
            m = np.mean([i.line_length for i in self.object_list])
            for i in self.object_list:
                i.difficulty = i.dist_to_segment(m, -1000, m, 1000)
                i.category = "A" if i.line_length < m else "B"

        with open(rospkg.RosPack().get_path('cognative_collaboration') + '/config/raw_positions.yaml', 'r') as stream:
            params = yaml.load(stream)
            self.grid_positions = params['grid_position']
            self.natural_position = params['natural']
            self.speech = params['speech']

    def robot_setup(self):
        baxter_interface.RobotEnable().enable()
        self.robot = baxter_interface.Limb('right')
        self.robot.set_joint_position_speed(0.5)
        self.talk = actionlib.SimpleActionClient('/speak', maryttsAction)

    def move_to(self, x, y):
        box = [i for i in self.object_list if i.grid_position_x == x and i.grid_position_y == y][0]
        self.robot.move_to_joint_positions(self.natural_position)
        self.talk.wait_for_server()
        self.talk.send_goal(maryttsGoal(text=self.process_speech(box.category)))
        self.robot.move_to_joint_positions(self.grid_positions[x][y])
        self.talk.wait_for_result()
        self.robot.move_to_joint_positions(self.natural_position)

    def process_speech(self, cat):
        return random.choice(self.speech).replace('{{category}}', 'AY' if cat == 'A' else 'BEE')

    def callback(self, data):
        if data.data == 'quit':
            rospy.signal_shutdown("quitting")
        else:            
            data = data.data.split(',')
            self.move_to(int(data[0]), int(data[0]))

if __name__ == '__main__':

    rospy.init_node("cognative_collaboration", anonymous=False, log_level=rospy.DEBUG)
    c = Controller()
    c.load_config()
    c.robot_setup()
    rospy.Subscriber("baxter_point", String, c.callback)
    # g = Gui(c.object_list)
    # g.show()
    rospy.spin()
