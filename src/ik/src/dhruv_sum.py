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

def callback(poser):
    global latest_pose
    latest_pose = poser

def main():
    global latest_pose
    robo = "sawyer"
    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    #rospy.init_node('service_query')
    arm = 'left'
    #left_gripper = robot_gripper.Gripper('left')
    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    if robo == 'sawyer':
        arm = 'right'
    while not rospy.is_shutdown():
        
        #raw_input('Press [ Enter ]: ')
        # Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = arm + "_arm"

        # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
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
    rospy.init_node('dhruv_sum_ik', anonymous = True)
    rospy.Subscriber('/sawyeet/des_end', PoseStamped, callback)
    r = rospy.Rate(15)
    #initialize()
    while not rospy.is_shutdown():
        main()
        r.sleep()

# Python's syntax for a main() method
if __name__ == '__main__':
    listener()