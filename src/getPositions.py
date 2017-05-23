import random

import actionlib

import baxter_interface

from mary_tts.msg import maryttsAction, maryttsGoal

import rospkg

import rospy

import yaml


def process_speech(cat):
    return random.choice(speech).replace('{{catagory}}', cat)

rospy.init_node(
    "cognative_collaboration",
    anonymous=False,
    log_level=rospy.DEBUG
)
robotEnable = baxter_interface.RobotEnable()
robotEnable.enable()
limb = baxter_interface.Limb('right')
limb.set_joint_position_speed(0.5)
# angles = limb.joint_angles()

with open(rospkg.RosPack().get_path('cognative_collaboration') + '/config/raw_positions.yaml', 'r') as stream:
    params = yaml.load(stream)
    grid_positions = params['grid_position']
    natural = params['natural']
    speech = params['speech']

limb.move_to_joint_positions(natural)
client = actionlib.SimpleActionClient('/speak', maryttsAction)
client.wait_for_server()
client.send_goal(maryttsGoal(text=process_speech('AE')))
limb.move_to_joint_positions(grid_positions[random.randint(0, 4)][random.randint(0, 4)])
client.wait_for_result()
