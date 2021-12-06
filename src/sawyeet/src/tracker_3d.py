#!/usr/bin/env python

from __future__ import print_function

from collections import deque

import cv2
import message_filters
import numpy as np
import rospy
import tf

from tracking_utils import get_centroid
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo, Image


def get_camera_matrix(camera_info_msg):
    return np.array(camera_info_msg.K).reshape(3, 3)


class PointcloudProcess:
    def __init__(self, points_sub_topic, image_sub_topic, cam_info_topic, points_pub_topic):
        self.messages = deque([], 5)

        # Create the subscribers
        points_sub = message_filters.Subscriber(points_sub_topic, Image)
        image_sub = message_filters.Subscriber(image_sub_topic, Image)
        caminfo_sub = message_filters.Subscriber(cam_info_topic, CameraInfo)

        self._bridge = CvBridge()
        self.listener = tf.TransformListener()
        
        # Publishers
        self.points_pub = rospy.Publisher(points_pub_topic, Point, queue_size=10)
        
        # Time sync
        ts = message_filters.ApproximateTimeSynchronizer([points_sub, image_sub, caminfo_sub],
                                                          1, 0.1, allow_headerless=True)
        ts.registerCallback(self.callback)


    def callback(self, points_msg, image, info):
        # Do our stuff here i think, and publish here?
        try:
            intrinsic_matrix = get_camera_matrix(info)
            rgb_image = self._bridge.imgmsg_to_cv2(image)
            bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
            depths = self._bridge.imgmsg_to_cv2(points_msg, desired_encoding='passthrough')
        except Exception as e:
            rospy.logerr(e)
            return      
        
        self.messages.appendleft((depths, bgr_image, intrinsic_matrix))
            
    def publish_once_from_queue(self):
        if self.messages:
            depths, image, intrinsic_matrix = self.messages.pop()

            tracking = np.zeros((image.shape[0],image.shape[1],3), np.uint8)
            center, points = get_centroid(image)
            if center:

                image[max(0,center[1]-20):min(image.shape[0], center[1]+20), max(0,center[0]-20):min(image.shape[1], center[0]+20)] = (0,255,0)

                depth = 0
                total = 0
                print("center depth", depths[center[1], center[0]])
                for point_array in points:
                    for point in point_array:
                        depth += depths[point[1], point[0]]
                        total += 1
                if total != 0:
                    avg_depth = depth / total * 0.001

                homog = np.array([center[0], center[1], 1])
                homog_X = depths[center[1], center[0]] * np.dot(np.linalg.inv(intrinsic_matrix), homog) * 0.001
                homog_X += np.array([0.0106, 0.0175, 0.0125]) # WRT center of realsense

                # Convert to sawyers base
                R_real_to_saw = np.array([
                    [1, 0, 0],
                    [0, 1, 0],
                    [0, 0, 1],
                ])

                P_real_to_saw = np.array([0.0106, 0.0175, 0.0125])

                point_sawyer = np.dot(R_real_to_saw, homog_X) + P_real_to_saw
                point = Point()

                point.x = -point_sawyer[0]
                point.y = -point_sawyer[1]
                point.z = point_sawyer[2]

                # point.x = 0
                # point.y = 0
                # point.z = 1.0

                print(point)

                self.points_pub.publish(point)

            cv2.imshow("depth", depths)
            cv2.imshow("image", image)
            #cv2.imshow("tracking", tracking)
            cv2.waitKey(1)
        

def main():
    CAM_INFO_TOPIC = '/camera/color/camera_info'
    RGB_IMAGE_TOPIC = '/camera/color/image_raw'
    ALIGNED_DEPTH_TOPIC = '/camera/aligned_depth_to_color/image_raw'
    POINTS_PUB_TOPIC = '/sawyeet/ball_coords'

    # ALIGNED_DEPTH_TOPIC = '/camera/depth/image_rect_raw' #MODIFY TO ALIGNED

    rospy.init_node('sawyeet_tracker')
    process = PointcloudProcess(ALIGNED_DEPTH_TOPIC, RGB_IMAGE_TOPIC, CAM_INFO_TOPIC, POINTS_PUB_TOPIC)
    r = rospy.Rate(30)

    while not rospy.is_shutdown():
        process.publish_once_from_queue()
        r.sleep()

if __name__ == '__main__':
    main()
