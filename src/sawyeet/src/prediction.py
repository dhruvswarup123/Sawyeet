#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
import numpy as np

pub = rospy.Publisher('des_end', Point, queue_size=10)

prev_x = []
prev_y = []
prev_z = []

des_xf = 0.0
des_yf = 0.2 #Needs to be modified based on the plane that we fit. 
des_zf = 0.0
num_frames = 10

def perpendicullar_fit(prev_p, prev_q):
    qp = np.polyfit(prev_p[-num_frames:], prev_q[-num_frames:], deg = 2)
    return (qp[0]*des_yf**2 + qp[1]*des_yf + qp[2]) 

def parallel_fit(prev_p, prev_q):
    qp = np.polyfit(prev_p[-num_frames:], prev_q[-num_frames:], deg = 2)
    return (qp[0]*des_zf**2 + qp[1]*des_zf + qp[2]) 

def callback(data):
    prev_x.append(data.x)
    prev_y.append(data.y)
    prev_z.append(data.z)
    prev_x = prev_x[-num_frames:]
    prev_y = prev_y[-num_frames:]
    prev_z = prev_z[-num_frames:]

    des_zf = perpendicular_fit(prev_y, prev_z)
    des_xf = parallel_fit(prev_y, prev_x)

    point = Point()
    point.x = des_xf
    point.y = des_yf
    point.z = des_zf
    pub.publish(point)


def listener():
    rospy.init_node('ball_prediction', anonymous=True)
    rospy.Subscriber('centroids', Point, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()