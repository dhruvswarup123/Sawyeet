#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
import numpy as np
import math

pub = rospy.Publisher('/sawyeet/des_end', PoseStamped, queue_size=10)

prev_time = None
curr_time = None

prev_x = []
prev_y = []
prev_z = []

des_xf = 0.0
des_yf = 0.0  
des_zf = 0.0 #Needs to be modified based on the plane that we fit.
num_frames = 30
index = 0

def perpendicular_fit(prev_p, prev_q):
    qp = np.polyfit(prev_p[-num_frames:], prev_q[-num_frames:], deg = 2)
    return (qp[0]*des_zf**2 + qp[1]*des_zf + qp[2]) 

def parallel_fit(prev_p, prev_q):
    qp = np.polyfit(prev_p[-num_frames:], prev_q[-num_frames:], deg = 2)
    return (qp[0]*des_yf**2 + qp[1]*des_yf + qp[2]) 

def callback(data):
    global prev_x, prev_y, prev_z, index, prev_time, curr_time
    index = index + 1
    if index % num_frames == 0:
        prev_time = curr_time
    dt = 0
    curr_time = rospy.Time.now().to_sec()
    prev_x.append(data.x)
    prev_y.append(data.y)
    prev_z.append(data.z)
    #prev_x = prev_x[-num_frames:]
    #prev_y = prev_y[-num_frames:]
    #prev_z = prev_z[-num_frames:]

    # des_yf = perpendicular_fit(prev_z, prev_y)
    # des_xf = parallel_fit(prev_y, prev_x)

    # des_pose = PoseStamped()
    # des_pose.pose.position.x = des_xf
    # des_pose.pose.position.y = des_yf
    # des_pose.pose.position.z = des_zf
    # roll = 0.0
    # pitch = math.atan((des_zf - prev_z[0])/(des_yf - prev_y[0])) 
    # yaw = math.atan((des_xf - prev_x[0])/(des_yf - prev_y[0]))
    # quat = quaternion_from_euler(roll, pitch, yaw)
    # des_pose.pose.orientation.x = quat[0]
    # des_pose.pose.orientation.y = quat[1]
    # des_pose.pose.orientation.z = quat[2]
    # des_pose.pose.orientation.w = quat[3]
    # #print(des_xf, des_yf, des_zf)
    # des_pose.pose.position.x = 0.8
    # des_pose.pose.position.y = 0.05
    # des_pose.pose.position.z = 0
    # #print(des_pose.pose.position.x)
    # pub.publish(des_pose)

def initialize():
    global prev_time, curr_time
    prev_time = rospy.Time.now().to_sec()
    curr_time = rospy.Time.now().to_sec()

def estimation():
    if len(prev_z) > num_frames:
        global curr_time, prev_time, prev_z, prev_y, prev_x
        dt = float(curr_time - prev_time)
        vz = (prev_z[-1] - prev_z[-num_frames])/dt
        vx = (prev_x[-1] - prev_x[-num_frames])/dt
        if vz != 0:
            t = -prev_z[-1]/vz
            print(t, dt)
            vy = (prev_y[-1] - prev_y[-num_frames] + 9.81*t**2/2.)/t
            index = 0
            print(vx, vy, vz)

            print(prev_x[-1] + t*vx, prev_y[-1] + t*vy - 9.81*t**2/2., prev_z[-1] + vz*t)

def listener():
    rospy.init_node('ball_prediction', anonymous=True)
    initialize()
    rospy.Subscriber('/sawyeet/ball_coords', Point, callback)
    rate = rospy.Rate(15)
    while not rospy.is_shutdown():
        prediction = estimation()
        rate.sleep()


if __name__ == '__main__':
    listener()