#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
import numpy as np
import math

pub = rospy.Publisher('/sawyeet/des_end', PoseStamped, queue_size=10)

prev_x = []
prev_y = []
prev_z = []

des_xf = 0.0
des_yf = 0.0  
des_zf = 0.0 #Needs to be modified based on the plane that we fit.
num_frames = 30


def perpendicular_fit(prev_p, prev_q):
    qp = np.polyfit(prev_p[-num_frames:], prev_q[-num_frames:], deg = 2)
    return (qp[0]*des_zf**2 + qp[1]*des_zf + qp[2]) 

def parallel_fit(prev_p, prev_q):
    qp = np.polyfit(prev_p[-num_frames:], prev_q[-num_frames:], deg = 2)
    return (qp[0]*des_yf**2 + qp[1]*des_yf + qp[2]) 

def callback(data):
    global prev_x, prev_y, prev_z
    prev_x.append(data.x)
    prev_y.append(data.y)
    prev_z.append(data.z)
    prev_x = prev_x[-num_frames:]
    prev_y = prev_y[-num_frames:]
    prev_z = prev_z[-num_frames:]

    des_yf = perpendicular_fit(prev_z, prev_y)
    des_xf = parallel_fit(prev_y, prev_x)

    des_pose = PoseStamped()
    des_pose.pose.position.x = des_xf
    des_pose.pose.position.y = des_yf
    des_pose.pose.position.z = des_zf
    roll = 0.0
    pitch = math.atan((des_zf - prev_z[0])/(des_yf - prev_y[0])) 
    yaw = math.atan((des_xf - prev_x[0])/(des_yf - prev_y[0]))
    quat = quaternion_from_euler(roll, pitch, yaw)
    des_pose.pose.orientation.x = quat[0]
    des_pose.pose.orientation.y = quat[1]
    des_pose.pose.orientation.z = quat[2]
    des_pose.pose.orientation.w = quat[3]
    print(des_xf, des_yf, des_zf)
    des_pose.pose.position.x = 0.8
    des_pose.pose.position.y = 0.05
    des_pose.pose.position.z = 0
    print(des_pose.pose.position.x)
    pub.publish(des_pose)


def listener():
    rospy.init_node('ball_prediction', anonymous=True)
    rospy.Subscriber('/sawyeet/ball_coords', Point, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()