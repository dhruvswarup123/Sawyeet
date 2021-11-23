#!/usr/bin/env python
"""Skeleton code for Lab 6
Course: EECS C106A, Fall 2019
Author: Amay Saxena

This file implements a ROS node that subscribes to topics for RGB images,
pointclouds, and camera calibration info, and uses the functions you
implemented to publish a segmented pointcloud to the topic /segmented_points.

Once you are confident in your implementation in image_segmentation.py and
pointcloud_segmentation.py, run this file to begin publishing a segmented
pointcloud.
"""

from __future__ import print_function
from collections import deque

import rospy
import message_filters
import ros_numpy
import tf

from sensor_msgs.msg import Image, CameraInfo, PointCloud2

import numpy as np
import cv2

from cv_bridge import CvBridge
from ball_tracker import get_centroid


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
        self.points_pub = rospy.Publisher(points_pub_topic, PointCloud2, queue_size=10)
        self.image_pub = rospy.Publisher('segmented_image', Image, queue_size=10)
        
        # Time sync
        ts = message_filters.ApproximateTimeSynchronizer([points_sub, image_sub, caminfo_sub],
                                                          10, 0.1, allow_headerless=True)
        ts.registerCallback(self.callback)


    def callback(self, points_msg, image, info):
        # Do our stuff here i think, and publish here?
        try:
            intrinsic_matrix = get_camera_matrix(info)
            rgb_image = self._bridge.imgmsg_to_cv2(image)
            bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
            depths = self._bridge.imgmsg_to_cv2(points_msg)
        except Exception as e:
            rospy.logerr(e)
            return      
        
        self.messages.appendleft((depths, bgr_image, intrinsic_matrix))
            
    def publish_once_from_queue(self):
        if self.messages:
            depths, rgb_image, intrinsic_matrix = self.messages.pop()
            cv2.imshow("depth", depths)
            cv2.imshow("image", rgb_image)
            cv2.waitKey(1)
            # print(depths.shape)
            # print(rgb_image.shape)

            center, points = get_centroid(rgb_image)
            if center:
                depth = 0
                total = 0
                for point_array in points:
                    for point in point_array:
                        depth += depths[min(point[0], depths.shape[0]-1), min(point[1], depths.shape[1]-1)]
                        total += 1
                if total != 0:
                    avg_depth = depth / total * 0.001
                
                homog = np.array([center[0], center[1], 1])
                homog_X = avg_depth * np.dot(np.linalg.inv(intrinsic_matrix), homog)
                homog_X += np.array([0.0106, 0.0175, 0.0125]) # WRT center of realsense
                
                print(homog_X)
            


        ########################


    '''
    /camera/color/camera_info:          sensor_msgs/CameraInfo
    /camera/color/image_raw:            sensor_msgs/Image           --- THIS ONE
    /camera/depth/color/points:         sensor_msgs/PointCloud2
    /camera/depth/image_rect_raw:       sensor_msgs/Image
    /camera/depth/metadata:             realsense2_camera/Metadata
    /camera/aligned_depth_to_color/image_raw     sensor_msgs/Image --- DEPTH
    '''

def main():
    CAM_INFO_TOPIC = '/camera/color/camera_info'
    RGB_IMAGE_TOPIC = '/camera/color/image_raw'
    ALIGNED_DEPTH_TOPIC = '/camera/depth/image_rect_raw' #MODIFY TO ALIGNED
    # ALIGNED_DEPTH_TOPIC = '/camera/aligned_depth_to_color/image_raw'
    POINTS_PUB_TOPIC = 'segmented_points'

    rospy.init_node('realsense_listener')
    process = PointcloudProcess(ALIGNED_DEPTH_TOPIC, RGB_IMAGE_TOPIC, CAM_INFO_TOPIC, POINTS_PUB_TOPIC)
    r = rospy.Rate(15)

    while not rospy.is_shutdown():
        process.publish_once_from_queue()
        r.sleep()

if __name__ == '__main__':
    main()
