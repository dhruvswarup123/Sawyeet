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
final_xy = None

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
    (0.087, -0.020): {'right_j6': 1.0524443359375, 'right_j5': 0.754802734375, 'right_j4': 0.754767578125, 'right_j3': 1.5055888671875, 'right_j2': -0.5154580078125, 'right_j1': -0.425685546875, 'right_j0': 0.2074306640625},
    (0.059, 0.301): {'right_j6': 0.976515625, 'right_j5': 1.3535986328125, 'right_j4': 0.8579912109375, 'right_j3': 1.366412109375, 'right_j2': -1.071869140625, 'right_j1': -0.6286982421875, 'right_j0': 0.3392236328125},
    (0.019, -0.273): {'right_j6': 0.4086533203125, 'right_j5': 0.7073935546875, 'right_j4': 1.8604873046875, 'right_j3': 1.453560546875, 'right_j2': -0.7272197265625, 'right_j1': 0.31555859375, 'right_j0': 0.39858984375},
    (-0.241, -0.009): {'right_j6': 0.000306640625, 'right_j5': 1.6335859375, 'right_j4': 2.0639892578125, 'right_j3': 1.544904296875, 'right_j2': -1.554279296875, 'right_j1': 0.4999326171875, 'right_j0': -0.09034765625},
    (-0.251, 0.253): {'right_j6': -0.370611328125, 'right_j5': 2.2945869140625, 'right_j4': 1.8677431640625, 'right_j3': 1.9248349609375, 'right_j2': -2.189185546875, 'right_j1': 0.445365234375, 'right_j0': -0.3913232421875},
    (-0.320, -0.256): {'right_j6': 0.2523154296875, 'right_j5': 1.61895703125, 'right_j4': 2.9698857421875, 'right_j3': 1.498650390625, 'right_j2': -2.3183740234375, 'right_j1': 1.2897109375, 'right_j0': -0.613625},
    (0.251, 0.050): {'right_j6': 0.9263046875, 'right_j5': 2.395021484375, 'right_j4': 2.7725546875, 'right_j3': 1.9592509765625, 'right_j2': -2.761234375, 'right_j1': 1.0036767578125, 'right_j0': -0.0825205078125},
    (0.269, 0.253) : {'right_j6': 0.567470703125, 'right_j5': 2.73820703125, 'right_j4': 2.2549423828125, 'right_j3': 1.9442177734375, 'right_j2': -2.77762109375, 'right_j1': 0.5726181640625, 'right_j0': -0.020119140625},
    (0.252, -0.271): {'right_j6': 1.1760283203125, 'right_j5': 1.74505859375, 'right_j4': 2.9728037109375, 'right_j3': 1.4405576171875, 'right_j2': -2.8806767578125, 'right_j1': 1.2680810546875, 'right_j0': -0.159740234375}
} 
final_joint_angles = eepos_real_to_angles.values()[0] # For the actual movement

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
    global final_xy
    latest_pose = poser

    currpos = np.array([poser.pose.position.x, poser.pose.position.y])

    closest_joint_angles = eepos_to_angles[(0, 0)]
    closest_dist = float('inf')
    closest_xy = float('inf')

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
            closest_xy = coords

    # final_joint_angles = angles_to_dicct(closest_joint_angles)
    final_joint_angles = closest_joint_angles
    final_xy = closest_xy

def main():
    global final_xy
    global final_joint_angles

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
    print(right.joint_angles())

    # right.move_to_neutral(timeout=5.0, speed=1)
    right.set_joint_position_speed(speed=0.6)
    rospy.sleep(2)

    #return

    r = rospy.Rate(15)
    while not rospy.is_shutdown():
        counter = 0
        print(final_xy)

        while(counter <= 500):   
            right.set_joint_position_speed(speed=1)
            right.set_joint_positions(final_joint_angles)
            rospy.sleep(0.001)
            counter += 1

        r.sleep()
        print("Moved...")
    
    print("Done.")


if __name__ == '__main__':
    rospy.Subscriber('/sawyeet/des_end', PoseStamped, callback)
    main()