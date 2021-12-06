#!/usr/bin/env python
import rospy
import intera_interface
from intera_interface import CHECK_VERSION
from geometry_msgs.msg import PoseStamped

import numpy as np

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
    (-0.572, 0.297): {'right_j6': 1.680861328125, 'right_j5': 1.3232978515625, 'right_j4': 0.318568359375, 'right_j3': 1.0760712890625, 'right_j2': -0.55458984375, 'right_j1': -0.7719267578125, 'right_j0': -0.8749248046875},
    ( -0.645, -0.064): {'right_j6': 1.40146875, 'right_j5': 1.1433193359375, 'right_j4': 0.662890625, 'right_j3': 0.4694990234375, 'right_j2': -0.6768037109375, 'right_j1': 0.08253515625, 'right_j0': -0.909541015625},
    (0.482, 0.236): {'right_j6': 4.3019248046875, 'right_j5': 1.34025, 'right_j4': 1.0057919921875, 'right_j3': 1.0442998046875, 'right_j2': -1.2243447265625, 'right_j1': -0.2824228515625, 'right_j0': 0.900169921875},
} 

def angles_to_dicct(angles):
    dicct_map = {
        'right_j0': angles[0], 
        'right_j1': angles[1], 
        'right_j2': angles[2], 
        'right_j3': angles[3], 
        'right_j4': angles[4], 
        'right_j5': angles[5], 
        'right_j6': angles[6]
        }
    
    return dicct_map

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

    # final_joint_angles = angles_to_dicct(closest_joint_angles)
    final_joint_angles = closest_joint_angles

def main():
    rospy.init_node("hardcoded_sawyeet")
    rs = intera_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    def clean_shutdown():
        print("\nExiting example...")
        if not init_state:
            print("Disabling robot...")
            rs.disable()
    rospy.on_shutdown(clean_shutdown)

    print("Enabling robot... ")
    rs.enable()
    right = intera_interface.Limb('right')
    # print(right.joint_angles())
    # return

    right.move_to_neutral(timeout=5.0, speed=1)
    right.set_joint_position_speed(speed=1)

    # return

    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        counter = 0
        print(final_joint_angles)

        while(counter <= 500):   
            # right.set_joint_positions(final_joint_angles)
            # rospy.sleep(0.001)
            counter += 1

        r.sleep()
        print("Moved...")
    
    print("Done.")


if __name__ == '__main__':
    rospy.Subscriber('/sawyeet/des_end', PoseStamped, callback)
    main()