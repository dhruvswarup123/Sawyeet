#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
import sys
from baxter_interface import gripper as robot_gripper

latest_pose = PoseStamped()

# Hardcode these
bottom_left_coord_xy = np.array([0, 0]) # in irl coords
paddle_size = np.array([0.1, 0.2]) # width, height
final_joint_angles = None # For the actual movement

# Here 0,0 is the bottom left square
eepos_to_angles = {
    (0, 0): [],
    (0, 1): [],
    (0, 2): [],
    (1, 0): [],
    (1, 1): [],
    (1, 2): [],
    (2, 0): [],
    (2, 1): [],
    (2, 2): [],
} 

eepos_real_to_angles = {
    (1.1, 2.3): [],
    (0, 1.4): [],
} 

def callback(poser):
    global latest_pose
    global final_joint_angles
    latest_pose = poser

    currpos = np.array([poser.pose.position.x, poser.pose.position.y])

    closest_joint_angles = eepos_to_angles[(0, 0)]
    closest_dist = float('inf')

    # for block_coords, joint_angles in eepos_to_angles.items():
    #     block_coords_np = np.array(block_coords)
    #     real_block_coords = bottom_left_coord_xy + np.multiply(block_coords_np, paddle_size)
    #     dist = np.linalg.norm(real_block_coords - currpos)

    #     if (dist < closest_dist):
    #         closest_joint_angles = joint_angles
    #         closest_dist = dist

    for coords, joint_angles in eepos_real_to_angles.items():
        real_coords_np = np.array(coords)
        dist = np.linalg.norm(real_coords_np - currpos)

        if (dist < closest_dist):
            closest_joint_angles = joint_angles
            closest_dist = dist

    final_joint_angles = closest_joint_angles


def main():
    global latest_pose
    robo = "sawyer"
    rospy.wait_for_service('compute_ik')
    arm = 'right'
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

    while not rospy.is_shutdown():
        request = GetPositionIKRequest()
        request.ik_request.group_name = arm + "_arm"

        link = arm + "_gripper"
        if robo == 'sawyer':
            link += '_tip'

        request.ik_request.ik_link_name = link
        request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"
        
        # Set the desired orientation for the end effector HERE
        request.ik_request.pose_stamped.pose.position.x = 0.6
        request.ik_request.pose_stamped.pose.position.y = latest_pose.pose.position.x
        request.ik_request.pose_stamped.pose.position.z = latest_pose.pose.position.y       
        request.ik_request.pose_stamped.pose.orientation.x = 1
        request.ik_request.pose_stamped.pose.orientation.y = 0
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w =  0

        
        try:
            # Send the request to the service
            response = compute_ik(request)
            
            # Print the response HERE
            print(response)
            group = MoveGroupCommander(arm + "_arm")

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)

            # Plan IK and execute
            group.go()

            print('Calibrating...')
            #left_gripper.calibrate()
            #rospy.sleep(2.0)

            print('Closing...')
            #rospy.sleep(1.0)

            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

def listener():
    global latest_pose
    rospy.init_node('sawyeet_hardcoded_ik', anonymous = True)
    rospy.Subscriber('/sawyeet/des_end', PoseStamped, callback)
    r = rospy.Rate(15)
    #initialize()
    while not rospy.is_shutdown():
        main()
        r.sleep()

# Python's syntax for a main() method
if __name__ == '__main__':
    listener()