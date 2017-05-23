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
# QListWidgetItem, QIcon
from PyQt4.QtGui import QApplication, QDialog, QGridLayout, QHBoxLayout, QListWidget, QPushButton, QVBoxLayout
from PyQt4 import QtCore
import actionlib
import baxter_interface
from mary_tts.msg import maryttsAction, maryttsGoal
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt
import numpy as np
import rospkg
import rospy
# import tf
# from geometry_msgs.msg import # PoseStamped Pose, Point, Quaternion, TransformStamped
# from tracking.msg import labelledPose, labelledPose_array
import yaml

import rospy
from std_msgs.msg import String

# ==================================================

class Model(QtCore.QAbstractListModel):
    def __init__(self, *args, **kwargs):
        QtCore.QAbstractListModel.__init__(self, *args, **kwargs)
        self.items = []

    def rowCount(self, parent=QtCore.QModelIndex()):
        return len(self.items)

    def data(self, index, role=QtCore.Qt.DisplayRole):
        if index.isValid() is True:
            if role == QtCore.Qt.DisplayRole:
                return QtCore.QVariant(self.items[index.row()])
            elif role == QtCore.Qt.ItemDataRole:
                return QtCore.QVariant(self.items[index.row()])
        return QtCore.QVariant()

    def itemsAdded(self, items):
        # insert items into their sorted position
        items = sorted(items)
        row = 0
        while row < len(self.items) and len(items) > 0:
            if items[0] < self.items[row]:
                self.items[row:row] = [items.pop(0)]
                row += 1
            row += 1
        # add remaining items to end of list
        if len(items) > 0:
            self.items.extend(items)

    def itemsRemoved(self, items):
        # remove items from list
        for item in items:
            for row in range(0, len(self.items)):
                if self.items[row] == item:
                    self.items.pop(row)
                    break

# ==================================================

class Box(QPushButton):

    def __init__(self, parent=None):
        super(Box, self).__init__()
        self.box_id = None
        self.tracking_ids_back = -1
        self.tracking_ids_right = -1
        self.tracking_ids_left = -1
        self.line_length = -1
        self.line_direction = -1
        self.grid_position = (-1, -1)
        self.category = "A"
        self.selected = False
        self.on_grid = True
        self.on_left_side_panel = not self.on_grid
        self.on_right_side_panel = not self.on_grid
        self.setFixedWidth(80)
        self.setFixedHeight(80)

    def dist_to_segment(self, ax, ay, bx, by):

        """
        Compute the minimum distance between a point (self.x, self.y) and a line segment
        :param ax: endpoint 1, x-coordinate
        :param ay: endpoint 1, y-coordinate
        :param bx: endpoint 2, x-coordinate
        :param by: endpoint 2, y-coordinate
        :return: minimum distance between point and line segment
        """
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


class Gui(QDialog):
    trigger = QtCore.pyqtSignal()
    colours = ['red', 'green', 'blue', 'yellow', 'purple', 'orange']
    left_side_width = 2
    right_side_width = 2
    grid_width = 5
    grid_height = 5
    object_list = list()

    def handle_trigger(self):
        for i in range(self.gridLayout.count()):
            
            self.gridLayout.itemAt(0).widget().setStyleSheet('background:blue;')
        
    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        self.object_has_moved(self.object_list[0].box_id, False, 'left')

    def object_has_moved(self, box_id, on_grid, position):
        for i in self.object_list:
            i.setStyleSheet("""
                background-color: %s;
                border:1px solid white;
                color: white;""" % (self.colours[i.on_grid]))

    def __init__(self, parent=None):
        super(Gui, self).__init__(parent)
        self.trigger.connect(self.handle_trigger)
        self.load_config()
        self.gui_setup()
        self.robot_setup()

    def robot_setup(self):
        baxter_interface.RobotEnable().enable()
        self.robot = baxter_interface.Limb('right')
        self.robot.set_joint_position_speed(0.5)
        self.talk = actionlib.SimpleActionClient('/speak', maryttsAction)

    def process_speech(self, cat):
        return random.choice(self.speech).replace('{{category}}', 'AE' if cat == 'A' else 'BEE')

    def move_to(self, x, y):
        box = [i for i in self.object_list if i.grid_position_x == x and i.grid_position_y == y][0]
        self.robot.move_to_joint_positions(self.natural_position)
        self.talk.wait_for_server()
        self.talk.send_goal(maryttsGoal(text=self.process_speech(box.category)))
        self.robot.move_to_joint_positions(self.grid_positions[x][y])
        # time.sleep(0.3)
        self.robot.move_to_joint_positions(self.natural_position)
        self.talk.wait_for_result()

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
                print i.difficulty
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

    def button_click(self):
        button = self.sender()
        idx = self.gridLayout.indexOf(button)
        self.trigger.emit()
        location = self.gridLayout.getItemPosition(idx)
        r, c = location[:2]
        self.move_to(r, c)
        box = [i for i in self.object_list if i.grid_position_x == r and i.grid_position_y == c][0]


    def plot(self):
        x = [i.line_length for i in self.object_list]
        y = [i.line_direction for i in self.object_list]
        # create an axis
        ax = self.figure.add_subplot(111)
        # discards the old graph
        ax.hold(False)
        colours = [self.colours[c.on_grid] for c in self.object_list]
        ax.scatter(x, y, color=colours)

        m = np.mean(x)

        ax.axvline(m, color='b', linestyle='dashed', linewidth=2)
        # refresh canvas
        self.canvas.draw()

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
            print self.grid_zero_zero_position
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
                    print body['grid_position']
                    b.world_position_x = self.grid_zero_zero_position[0] + (self.grid_cell_height * b.grid_position_x)
                    b.world_position_y = self.grid_zero_zero_position[1] + (self.grid_cell_width * b.grid_position_y)
                    b.world_position_z = self.grid_zero_zero_position[2]
                    self.object_list.append(b)
            # ============== Box calculations ======================
            m = np.mean([i.line_length for i in self.object_list])
            for i in self.object_list:
                i.difficulty = i.dist_to_segment(m, -1000, m, 1000)
                i.category = "A" if i.line_length < m else "B"
                i.setText("Cat %s\n%0.4f\n%s" % (i.category, i.difficulty, i.box_id))
                i.setStyleSheet("""
                    background-color: %s;
                    border:1px solid white;
                    color: white;""" % (self.colours[i.on_grid]))

        with open(rospkg.RosPack().get_path('cognative_collaboration') + '/config/raw_positions.yaml', 'r') as stream:
            params = yaml.load(stream)
            self.grid_positions = params['grid_position']
            self.natural_position = params['natural']
            self.speech = params['speech']


if __name__ == '__main__':

    rospy.init_node("cognative_collaboration", anonymous=False, log_level=rospy.DEBUG)
    app = QApplication(sys.argv)
    main = Gui()
    rospy.Subscriber("chatter", String, main.callback)
    main.show()
    sys.exit(app.exec_())
