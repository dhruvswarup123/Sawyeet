#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Point

import cv2
import imutils
import numpy as np

RADIUS_THRESHOLD = 10
COLOR = "red"


def get_mask(frame, color=COLOR):
    # Hue color wheel https://cvexplained.wordpress.com/2020/04/28/color-detection-hsv/

    if color == "red":
        # lower boundary RED color range values; Hue (0 - 10)
        lower1 = np.array([0, 100, 20])
        upper1 = np.array([10, 255, 255])
        
        # upper boundary RED color range values; Hue (160 - 180)
        lower2 = np.array([160,100,20])
        upper2 = np.array([179,255,255])

        lower_mask = cv2.inRange(frame, lower1, upper1)
        upper_mask = cv2.inRange(frame, lower2, upper2)

        full_mask = lower_mask + upper_mask

    elif color == "yellow":
        lower = (20, 80, 20)
        upper = (40, 255, 255)

        full_mask = cv2.inRange(frame, lower, upper)

    elif color == "blue":
        lower = (85, 80, 20)
        upper = (130, 255, 255)

        full_mask = cv2.inRange(frame, lower, upper)
    
    full_mask = cv2.erode(full_mask, None, iterations=2)
    full_mask = cv2.dilate(full_mask, None, iterations=2)
    full_mask = cv2.morphologyEx(full_mask, cv2.MORPH_OPEN, np.ones((30, 30), np.uint8))

    return full_mask


def get_centroid(frame):
    result = frame.copy()

    # Blur the frame and convert to hsv color space
    frame = cv2.GaussianBlur(frame, (15, 15), 0)
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Get mask that removes pixels that are not that color
    mask = get_mask(hsv_frame)
    
    # Apply mask
    result = cv2.bitwise_and(result, result, mask=mask)

    # find contours in the mask and initialize the current
	# (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None
    radius = 0
    
    if len(cnts) > 0:
        # find the largest contour in the mask
        c = max(cnts, key=cv2.contourArea)
        # compute the minimum enclosing circle and centroid
        # x, y is center of enclosing circle, NOT the centroid
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        if radius < RADIUS_THRESHOLD:
            return None

        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

    return center


pts = []
pub = rospy.Publisher('centroids', Point, queue_size=10)
def callback(data):
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    center = get_centroid(frame)

    if center:
        cv2.circle(frame, center, 5, (0, 0, 255), -1)
        pts.append(center)
        point = Point()
        point.x = center[0]
        point.y = center[1]
        pub.publish(point)

    # Draw the trail behind the object
    for i in range(1, len(pts)):
        if pts[i - 1] is None or pts[i] is None:
            continue

        if i > (len(pts) - 100):
            thickness = 5
            cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)             
        
    cv2.imshow("frame", frame)
    cv2.waitKey(1)


def listener():
    rospy.init_node('ball_tracker', anonymous=True)
    rospy.Subscriber("image_raw", Image, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()