#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
import numpy as np
import math

NUM_FRAMES = 30
G = 9.81
ALPHA = 0.5
IS_INITIALIZED = False

pub = rospy.Publisher('/sawyeet/des_end', PoseStamped, queue_size=1)
prev_state = np.zeros((6,1))
curr_state = np.zeros((6,1))
meas_state = np.zeros((6,1))
pred_state = np.zeros((6,1))
prev_time = None

def intialize(coords):
    global prev_state, prev_time, IS_INITIALIZED
    prev_time = rospy.Time.now().to_sec() 
    prev_state[0] = coords.x
    prev_state[1] = coords.y
    prev_state[2] = coords.z
    prev_state[3] = 0.
    prev_state[4] = 0.
    prev_state[5] = 0.
    curr_state = prev_state
    pred_state = prev_state
    IS_INITIALIZED = True
    return

def stateCallback(coords):
    global prev_state, prev_time, pred_state, curr_state, meas_state, IS_INITIALIZED, ALPHA, G
    curr_time = rospy.Time.now().to_sec() 
    dT = (curr_time - prev_time)
    prev_time = curr_time

    if float(dT) > 0.3 or not IS_INITIALIZED:
        print("Initializing/Re-Initializing")
        intialize(coords)
        
    else:
        #print(coords.x, float(prev_state[0]), float(prev_state[0]) - coords.x)

        meas_state = np.zeros((6,1))
        meas_state[0] = coords.x
        meas_state[1] = coords.y
        meas_state[2] = coords.z
        meas_state[3] = (coords.x - prev_state[0])/dT
        meas_state[4] = (coords.y - prev_state[1])/dT
        meas_state[5] = (coords.z - prev_state[2])/dT
        
        #print("##MEAS STATE START##")
        #print(meas_state)
        #print("##MEAS STATE END##")

        pred_state = np.zeros((6,1))
        pred_state[0] = prev_state[0] + prev_state[3] * dT
        pred_state[1] = prev_state[1] + prev_state[4] * dT - G * dT**2 / 2.
        pred_state[2] = prev_state[2] + prev_state[5] * dT 
        pred_state[3] = prev_state[3]
        pred_state[4] = prev_state[4] - G * dT
        pred_state[5] = prev_state[5]

        #print("##PRED STATE START########################")
        #print(pred_state)
        #print("##PRED STATE END########################")
        curr_state = pred_state + ALPHA * (meas_state - pred_state)

        #print("##CURR STATE START########################")
        #print(curr_state)
        #print("##CURR STATE END########################")


    # Data Process in Standard Frame
    ##################################################################################

        des_pose = PoseStamped()
        des_pose.pose.position.x = prev_state[0]
        des_pose.pose.position.y = prev_state[1]
        des_pose.pose.position.z = prev_state[2]
        roll = 0.0
        pitch = math.atan((curr_state[2] - prev_state[2])/(curr_state[1] - prev_state[1])) 
        yaw = math.atan((curr_state[0] - prev_state[0])/(curr_state[1] - prev_state[1]))
        quat = quaternion_from_euler(roll, pitch, yaw)

        prev_state = curr_state

    # Publish frames
    ###################################################################################
        
        des_pose.pose.orientation.x = 0.#quat[0]
        des_pose.pose.orientation.y = 0.#quat[1]
        des_pose.pose.orientation.z = 0.#quat[2]
        des_pose.pose.orientation.w = 1.#quat[3]
        #des_pose.pose.position.x = cur
        #des_pose.pose.position.y = des_yf
        #des_pose.pose.position.z = des_zf
        pub.publish(des_pose)
        
    ###################################################################################


def estimation(curr_state):
    global prev_state, meas_state, G
    #print("Estimating")
    #print("##CURR STATE START##")
    #print(curr_state)
    #print("##CURR STATE END##")
    pred_time = curr_state[2]/curr_state[5]
    #print(pred_time)
    pub_state = np.zeros(3)
    pub_state[0] = curr_state[0] + curr_state[3] * pred_time
    pub_state[1] = curr_state[1] + curr_state[4] * pred_time - G * pred_time**2 / 2. 
    print("CURRENT MEASUREMENT")
    meas = np.array([float(meas_state[0]), float(meas_state[1]), float(meas_state[2])])
    print(np.round(meas,2))
    print("FINAL PREDICTION")
    print(np.round(pub_state,2))
    return pub_state

def listener():
    global prev_time, curr_state
    rospy.init_node('ball_prediction', anonymous=True)
    prev_time = rospy.Time.now().to_sec()
    rospy.Subscriber('/sawyeet/ball_coords', Point, stateCallback)
    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        prediction = estimation(curr_state)
        rate.sleep()


if __name__ == '__main__':
    listener()