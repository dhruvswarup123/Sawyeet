#!/usr/bin/env python

import sys
ROBOT = "sawyer"

from intera_interface import Limb

import rospy
import numpy as np
import traceback
from geometry_msgs.msg import Point
from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped

from path_planner import PathPlanner
try:
    from controller import Controller
except ImportError:
    pass

Kp = 0.1 * np.array([0.4, 2, 1.7, 1.5, 2, 2, 3])
Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])

planner = PathPlanner("right_arm")

latest_pose = PoseStamped()

def callback(poser):
    global latest_pose
    latest_pose = poser

def move(latest_pose, c):

    try:

        goal_1 = PoseStamped()
        goal_1.header.frame_id = "base"

        #x, y, and z position
        goal_1.pose.position.x = 0.75
        goal_1.pose.position.y = (latest_pose.pose.position.x)
        goal_1.pose.position.z = (latest_pose.pose.position.y)

        
        #Orientation as a quaternion
        goal_1.pose.orientation.x = 0.0
        goal_1.pose.orientation.y = -1.0
        goal_1.pose.orientation.z = 0.0
        goal_1.pose.orientation.w = 0.0

        orien_const = OrientationConstraint()
        orien_const.link_name = "right_gripper"
        orien_const.header.frame_id = "base"
        orien_const.orientation.x = 0.0
        orien_const.orientation.y = -1.0
        orien_const.orientation.z = 0.0
        orien_const.orientation.w = 0.0
        orien_const.absolute_x_axis_tolerance = 0.2
        orien_const.absolute_y_axis_tolerance = 0.2
        orien_const.absolute_z_axis_tolerance = 0.2
        orien_const.weight = 1.0

        plan = planner.plan_to_pose(goal_1, [orien_const])
        
        if not c.execute_path(plan,10, False):
            raise Exception("Execution failed")
    
    except Exception as e:
        print e
        traceback.print_exc()


def initialize():
    raw_input("Press enter to initialize")
    try:
        x, y, z = 0.75, 0.05, 0.07
        goal_1 = PoseStamped()
        goal_1.header.frame_id = "base"

        #x, y, and z position
        goal_1.pose.position.x = x
        goal_1.pose.position.y = y
        goal_1.pose.position.z = z

        #Orientation as a quaternion
        goal_1.pose.orientation.x = 0.0
        goal_1.pose.orientation.y = -1.0
        goal_1.pose.orientation.z = 0.0
        goal_1.pose.orientation.w = 0.0

        # Might have to edit this . . . 
        plan = planner.plan_to_pose(goal_1, [])

        #raw_input("Press <Enter> to move the right arm to goal pose 1: ")
        if not planner.execute_plan(plan):
            raise Exception("Execution failed")
    except Exception as e:
        print e
        traceback.print_exc()

    raw_input("Press enter to start tracking")

def listener():
    global latest_pose
    rospy.init_node('sawyeet_ik', anonymous = True)
    c = Controller(Kp, Kd, Ki, Kw, Limb("right"))
    rospy.Subscriber('/sawyeet/des_end', PoseStamped, callback)
    r = rospy.Rate(15)
    initialize()
    while not rospy.is_shutdown():
        move(latest_pose, c)
        r.sleep()

if __name__ == '__main__':
    listener()