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
#0.65 = x
# eepos_real_to_angles = {
#     (-0.017, 0.050): {'right_j6': 0.4538935546875, 'right_j5': 1.41458984375, 'right_j4': 1.7936337890625, 'right_j3': 1.6538984375, 'right_j2': -1.36534765625, 'right_j1': 0.209646484375, 'right_j0': 0.359529296875},
#     (0.059, 0.301): {'right_j6': 0.976515625, 'right_j5': 1.3535986328125, 'right_j4': 0.8579912109375, 'right_j3': 1.366412109375, 'right_j2': -1.071869140625, 'right_j1': -0.6286982421875, 'right_j0': 0.3392236328125},
#     (0.019, -0.273): {'right_j6': 0.4086533203125, 'right_j5': 0.7073935546875, 'right_j4': 1.8604873046875, 'right_j3': 1.453560546875, 'right_j2': -0.7272197265625, 'right_j1': 0.31555859375, 'right_j0': 0.39858984375},
#     (-0.241, -0.009): {'right_j6': 0.000306640625, 'right_j5': 1.6335859375, 'right_j4': 2.0639892578125, 'right_j3': 1.544904296875, 'right_j2': -1.554279296875, 'right_j1': 0.4999326171875, 'right_j0': -0.09034765625},
#     (-0.251, 0.253): {'right_j6': -0.370611328125, 'right_j5': 2.2945869140625, 'right_j4': 1.8677431640625, 'right_j3': 1.9248349609375, 'right_j2': -2.189185546875, 'right_j1': 0.445365234375, 'right_j0': -0.3913232421875},
#     (-0.320, -0.256): {'right_j6': 0.2523154296875, 'right_j5': 1.61895703125, 'right_j4': 2.9698857421875, 'right_j3': 1.498650390625, 'right_j2': -2.3183740234375, 'right_j1': 1.2897109375, 'right_j0': -0.613625},
#     (0.251, 0.050): {'right_j6': 0.9263046875, 'right_j5': 2.395021484375, 'right_j4': 2.7725546875, 'right_j3': 1.9592509765625, 'right_j2': -2.761234375, 'right_j1': 1.0036767578125, 'right_j0': -0.0825205078125},
#     (0.269, 0.253) : {'right_j6': 0.567470703125, 'right_j5': 2.73820703125, 'right_j4': 2.2549423828125, 'right_j3': 1.9442177734375, 'right_j2': -2.77762109375, 'right_j1': 0.5726181640625, 'right_j0': -0.020119140625},
#     (0.252, -0.271): {'right_j6': 1.1760283203125, 'right_j5': 1.74505859375, 'right_j4': 2.9728037109375, 'right_j3': 1.4405576171875, 'right_j2': -2.8806767578125, 'right_j1': 1.2680810546875, 'right_j0': -0.159740234375},
#     (0.522, 0.270): {'right_j6': 1.4112138671875, 'right_j5': 2.460828125, 'right_j4': 2.6042861328125, 'right_j3': 1.432021484375, 'right_j2': -2.796470703125, 'right_j1': 0.392142578125, 'right_j0': 0.2803798828125},
#     (0.529, 0.014): {'right_j6': 1.47810546875, 'right_j5': 2.1024208984375, 'right_j4': 2.851724609375, 'right_j3': 1.5191689453125, 'right_j2': -2.7551435546875, 'right_j1': 0.8544521484375, 'right_j0': 0.30418359375},
#     (0.499, -0.266): {'right_j6': 1.54477734375, 'right_j5': 1.5748662109375, 'right_j4': 2.9750888671875, 'right_j3': 1.2560107421875, 'right_j2': -2.7505048828125, 'right_j1': 1.1523017578125, 'right_j0': 0.2597724609375},
# } 

eepos_real_to_angles = {
    (0.049, -0.039): {'right_j6': 0.5502353515625, 'right_j5': 1.529755859375, 'right_j4': 2.1438564453125, 'right_j3': 1.5061669921875, 'right_j2': -1.6607021484375, 'right_j1': 0.613080078125, 'right_j0': 0.354171875},
    (0.299, -0.008): {'right_j6': 0.9839912109375, 'right_j5': 1.8857880859375, 'right_j4': 2.4141337890625, 'right_j3': 1.47293359375, 'right_j2': -2.1419482421875, 'right_j1': 0.766607421875, 'right_j0': 0.397560546875},
    (-0.304, -0.032): {'right_j6': 0.227396484375, 'right_j5': 1.823970703125, 'right_j4': 2.4767392578125, 'right_j3': 1.4990830078125, 'right_j2': -2.1228994140625, 'right_j1': 0.82386328125, 'right_j0': -0.3972919921875},
    (-0.299, 0.299): {'right_j6': 0.056146484375, 'right_j5': 2.3042705078125, 'right_j4': 2.1164716796875, 'right_j3': 1.437958984375, 'right_j2': -2.48555078125, 'right_j1': 0.3087607421875, 'right_j0': -0.5735166015625},
    (-0.031, 0.312): {'right_j6': 0.1885869140625, 'right_j5': 2.4777548828125, 'right_j4': 2.0177421875, 'right_j3': 1.669373046875, 'right_j2': -2.581505859375, 'right_j1': 0.36177734375, 'right_j0': -0.2548662109375},
    (0.288, 0.321): {'right_j6': 0.7532470703125, 'right_j5': 2.5134404296875, 'right_j4': 2.2019814453125, 'right_j3': 1.5269833984375, 'right_j2': -2.681322265625, 'right_j1': 0.3187490234375, 'right_j0': 0.0886416015625},
    (0.302, -0.279): {'right_j6': 1.2918916015625, 'right_j5': 1.6838740234375, 'right_j4': 2.9753076171875, 'right_j3': 1.3362109375, 'right_j2': -2.758482421875, 'right_j1': 1.22214453125, 'right_j0': -0.029490234375},
    (-0.019, -0.202): {'right_j6': 0.8852353515625, 'right_j5': 1.7483642578125, 'right_j4': 2.9523779296875, 'right_j3': 1.6151513671875, 'right_j2': -2.6595673828125, 'right_j1': 1.279619140625, 'right_j0': -0.3661103515625},
    (-0.305, -0.301): {'right_j6': 0.5747265625, 'right_j5': 1.59361328125, 'right_j4': 2.9719638671875, 'right_j3': 1.3178564453125, 'right_j2': -2.734705078125, 'right_j1': 1.246546875, 'right_j0': -0.82759765625}
} 

final_joint_angles = None#eepos_real_to_angles[(0.049, -0.039)] # For the actual movement
center_pos = eepos_real_to_angles[(0.049, -0.039)]
# center_pos = eepos_real_to_angles[(-0.017, 0.050)]

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
    _ = raw_input("Press enter to start...")

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
    move_to_center(right)

    #return

    r = rospy.Rate(30)

    while not rospy.is_shutdown():
        if final_joint_angles is None:
            continue 
        counter = 0
        print(final_xy)

        while(counter <= 500):   
            right.set_joint_position_speed(speed=1.0)
            right.set_joint_positions(final_joint_angles)
            rospy.sleep(0.00001)
            counter += 1

        r.sleep()
    
    print("Done.")


def move_to_center(right):
    rospy.sleep(2)
    counter = 0
    print("centering...")

    while(counter <= 1500):   
        right.set_joint_position_speed(speed=0.6)
        right.set_joint_positions(center_pos)
        rospy.sleep(0.001)
        counter += 1
    
    rospy.sleep(1)


if __name__ == '__main__':
    rospy.Subscriber('/sawyeet/des_end', PoseStamped, callback)
    main()