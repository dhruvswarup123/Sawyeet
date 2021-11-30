#!/usr/bin/env python

import sys
assert sys.argv[1] in ("sawyer", "baxter")
ROBOT = sys.argv[1]

if ROBOT == "baxter":
    from baxter_interface import Limb
else:
    from intera_interface import Limb

import rospy
import numpy as np
import traceback
from geometry_msgs import Point
from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped

from path_planner import PathPlanner
try:
    from controller import Controller
except ImportError:
    pass

def callback(Point):

    planner = PathPlanner("right_arm")

    if ROBOT == "sawyer":
        Kp = 0.2 * np.array([0.4, 2, 1.7, 1.5, 2, 2, 3])
        Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
        Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
        Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])
    else:
        Kp = 0.45 * np.array([0.8, 2.5, 1.7, 2.2, 2.4, 3, 4])
        Kd = 0.015 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
        Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
        Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])


    ##
    ## Add the obstacle to the planning scene here
    ##
    # obst = PoseStamped()
    # obst.pose.position.x = 0.5
    # obst.pose.position.y = -0.5
    # obst.pose.position.z = 0.0

    # #Orientation as a quaternion
    # obst.pose.orientation.x = 0.0
    # obst.pose.orientation.y = 0.0
    # obst.pose.orientation.z = 0.0
    # obst.pose.orientation.w = 1.0
    # obst.header.frame_id = "base";


    # planner.add_box_obstacle([0.2,0.1,0.2],'obs',obst)


    # #Create a path constraint for the arm
    # #UNCOMMENT FOR THE ORIENTATION CONSTRAINTS PART
    orien_const = OrientationConstraint()
    orien_const.link_name = "right_gripper"
    orien_const.header.frame_id = "base"
    orien_const.orientation.x = 0.0
    orien_const.orientation.y = -1.0
    orien_const.orientation.z = 0.0
    orien_const.orientation.w = 0.0
    orien_const.absolute_x_axis_tolerance = 0.1
    orien_const.absolute_y_axis_tolerance = 0.1
    orien_const.absolute_z_axis_tolerance = 0.1
    orien_const.weight = 1.0

    c = Controller(Kp, Kd, Ki, Kw, Limb("right"))

    while not rospy.is_shutdown():

        while not rospy.is_shutdown():
            try:

                goal_1 = PoseStamped()
                goal_1.header.frame_id = "base"

                #x, y, and z position
                goal_1.pose.position.x = Point.position.x
                goal_1.pose.position.y = Point.position.y
                goal_1.pose.position.z = Point.position.z

                #Orientation as a quaternion
                goal_1.pose.orientation.x = 0.0
                goal_1.pose.orientation.y = -1.0
                goal_1.pose.orientation.z = 0.0
                goal_1.pose.orientation.w = 0.0

                # Might have to edit this . . .
                plan = planner.plan_to_pose(goal_1, orien_const)

                raw_input("Press <Enter> to move the right arm to goal pose 1: ")
                if not c.execute_path(plan,100,True):
                    raise Exception("Execution failed")
            except Exception as e:
                print e
                traceback.print_exc()
            else:
                break

def listener():
    rospy.init_node('end_effector_move', anonymous=True)
    rospy.Subscriber('/sawyeet/des_end', Point, callback)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('moveit_node')
    rospy.init_node('listener')
    listener()