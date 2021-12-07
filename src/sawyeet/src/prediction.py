#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from multiprocessing import Process
from multiprocessing.sharedctypes import Value
from ctypes import c_double
from time import sleep
import numpy as np
import math
import sys

###################################################################################
#CONSTANTS

NUM_FRAMES = 5
G = 9.8
X_N = float(sys.argv[1])
VX_N = float(sys.argv[2])
VY_N = float(sys.argv[3])
Y_N = float(sys.argv[4])
VZ_N = float(sys.argv[5])
DIST_Z = 60*2.54/100

###################################################################################
#GLOBAL VARIABLES

IS_INITIALIZED = False
pub = rospy.Publisher('/sawyeet/des_end', PoseStamped, queue_size=1)
prev_state = np.zeros((6,1))
curr_state = np.zeros((6,1))
meas_state = np.zeros((6,1))
pred_state = np.zeros((6,1))
prev_time = None
#prev_x = 0
#prev_y = 0
#prev_z = 0

xdata = []
ydata = []
zdata = []

curr_x = Value(c_double, 0)
curr_y = Value(c_double, 0)
curr_z = Value(c_double, 20*100/2.54)
published = Value('i', 0)
color = Value('i', 0)
last = Value('i', 0)

###################################################################################
#GRAPHING & ANIMATION

def runGraph(curr_x, curr_y, curr_z, published, color, last):
    def animate(i):
        if published.value == 0:
            xdata.append(curr_x.value)
            ydata.append(curr_y.value)
            zdata.append(curr_z.value)

            ax = plt.axes(projection='3d')

            if color.value == 0:
                last.value = last.value + 1
                ax.scatter3D(np.array(xdata)*100/2.54, -np.array(zdata)*100/2.54, np.array(ydata)*100/2.54, c ='g')
            else:
                ax.scatter3D(np.array(xdata[:last.value])*100/2.54, -np.array(zdata[:last.value])*100/2.54, np.array(ydata[:last.value])*100/2.54, c = 'g')
                ax.scatter3D(np.array(xdata[last.value:])*100/2.54, -np.array(zdata[last.value:])*100/2.54, np.array(ydata[last.value:])*100/2.54, c = 'r')

            ax.set_xlabel('x')
            ax.set_ylabel('z')
            ax.set_zlabel('y')
            ax.set_xlim([-2*100/2.54, 2*100/2.54])
            ax.set_ylim([-5*100/2.54, 2*100/2.54])
            ax.set_zlim([-2*100/2.54, 2*100/2.54])

    ani = FuncAnimation(plt.gcf(), animate, interval=1, repeat=False)
    plt.show()
    plt.close()

###################################################################################
#PREDICTED OUTPUT GRAPHING

def plot_predicted():
    global G, color, curr_x, curr_y, curr_z, curr_state, DIST_Z
    color.value = 1
    delta = 0.01
    time = 0
    while time < (-(DIST_Z+curr_state[2])/curr_state[5]):
        curr_x.value = curr_state[0] + curr_state[3] * time
        curr_y.value = curr_state[1] + curr_state[4] * time - G * time**2 / 2. 
        curr_z.value = curr_state[2] + curr_state[5] * time
        time += delta 
        sleep(0.1)

###################################################################################
#INITIALIZATIOon

def intialize(coords):
    global prev_state, prev_time, IS_INITIALIZED, curr_state, pred_state, prev_z, meas_state
    prev_time = rospy.Time.now().to_sec() 
    prev_state[0] = coords.x
    prev_state[1] = coords.y
    prev_state[2] = coords.z
    prev_state[3] = 0.
    prev_state[4] = 0.
    prev_state[5] = 0.
    prev_z = coords.y
    curr_state = prev_state
    pred_state = prev_state
    meas_state = prev_state
    IS_INITIALIZED = True
    return

###################################################################################
#STATE UPDATE AND KALMAN FILTER

def stateCallback(coords):
    global prev_state, prev_time, pred_state, curr_state, meas_state, IS_INITIALIZED, X_N, G, curr_x, curr_y, curr_z, Y_N, VX_N, VY_N, VZ_N, prev_z
    curr_time = rospy.Time.now().to_sec() 
    dT = (curr_time - prev_time)

    print("dt", dT)
    prev_time = curr_time

    if rospy.Time.now().to_sec() - prev_time > 0.3 or not IS_INITIALIZED:
        print("Initializing/Re-Initializing")
        intialize(coords)
        
    elif IS_INITIALIZED and abs(coords.x - meas_state[0]) < 0.5 and abs(coords.y - meas_state[1]) < 0.5 and abs(coords.z - meas_state[2]) < 0.5:
    #else:
        meas_state = np.zeros((6,1))
        meas_state[0] = coords.x
        meas_state[1] = coords.y
        meas_state[2] = coords.z
        meas_state[3] = (coords.x - prev_state[0])/dT
        meas_state[4] = (coords.y - prev_state[1])/dT
        meas_state[5] = (coords.z - prev_state[2])/dT

        curr_x.value = meas_state[0]
        curr_y.value = meas_state[1]
        curr_z.value = meas_state[2]

        pred_state = np.zeros((6,1))
        pred_state[0] = prev_state[0] + prev_state[3] * dT  
        pred_state[1] = prev_state[1] + prev_state[4] * dT - G * dT**2 / 2. 
        pred_state[2] = prev_state[2] + prev_state[5] * dT
        pred_state[3] = prev_state[3]
        pred_state[4] = prev_state[4] - G * dT
        pred_state[5] = prev_state[5]

        update_matrix = np.array([[X_N],[Y_N],[X_N],[VX_N],[VY_N],[VZ_N]])
        curr_state = pred_state + update_matrix * (meas_state - pred_state)
        #print(coords.y, prev_z)
        print("Velocities START")
        print(curr_state[3:])
        print("Velocities END")
        prev_state = curr_state

###################################################################################
#PUBLISH FUNCTION

def publish_pose(pred_state):
    des_pose = PoseStamped()
    des_pose.pose.position.x = pred_state[0] + 0.15
    des_pose.pose.position.y = pred_state[1] - 0.55
    des_pose.pose.position.z = pred_state[2]
    des_pose.pose.orientation.x = 0.
    des_pose.pose.orientation.y = 0.
    des_pose.pose.orientation.z = 0.
    des_pose.pose.orientation.w = 1.
    pub.publish(des_pose)
     
###################################################################################
#ESTIMATION FUNCTION -- ONCE BALL GOES OUT OF FRAME

def estimation():
    global prev_state, curr_state, meas_state, G, DIST_Z
    if rospy.Time.now().to_sec() - prev_time > 0.3 and curr_state[5] != 0:
        pred_time = -(DIST_Z+curr_state[2])/(curr_state[5])
        print(pred_time)
        #print(curr_state[3], curr_state[4], curr_state[5])
        pub_state = np.zeros(3)
        pub_state[0] = curr_state[0] + curr_state[3] * pred_time
        pub_state[1] = curr_state[1] + curr_state[4] * pred_time - G * pred_time**2 / 2. 
        meas = np.array([float(meas_state[0]), float(meas_state[1]), float(meas_state[2])])
        print(np.round(meas,2), np.round(pub_state,2)*100/2.54, np.round(curr_state[:3].reshape(-1),2))
        publish_pose(pub_state)
        return True
    return False

###################################################################################
#MAIN LISTENER

def listener():
    global prev_time, curr_state, curr_x, curr_y, curr_z, published, color
    rospy.init_node('ball_prediction', anonymous=True)
    prev_time = rospy.Time.now().to_sec()
    rospy.Subscriber('/sawyeet/ball_coords', Point, stateCallback)
    rate = rospy.Rate(30)
    p = Process(target=runGraph, args=(curr_x, curr_y, curr_z, published, color, last))
    p.daemon = True
    p.start()
    while not rospy.is_shutdown():
        received = estimation() 
        if received:
            plot_predicted()
            published.value = 1
            p.terminate()
            exit(0)
        rate.sleep()

###################################################################################

if __name__ == '__main__':
    listener()

###################################################################################