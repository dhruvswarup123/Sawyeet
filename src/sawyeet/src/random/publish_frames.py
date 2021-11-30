#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2


def talker(video_file):
    pub = rospy.Publisher('image_raw', Image, queue_size=10)
    rospy.init_node('usb_cam_temp', anonymous=True)
    rate = rospy.Rate(15)

    cap = cv2.VideoCapture(video_file)
    # Check if camera opened successfully
    if (cap.isOpened()== False): 
        print("Error opening video stream or file")

    bridge = CvBridge()
    while(cap.isOpened()):
        if rospy.is_shutdown():
            break

        ret, frame = cap.read()

        if ret == True:
            frame = bridge.cv2_to_imgmsg(frame)
            pub.publish(frame)
            
            rate.sleep()

            # Press Q on keyboard to  exit
            if cv2.waitKey(25) & 0xFF == ord('q'):
                break

        # Break the loop
        else: 
            break


if __name__ == '__main__':
    try:
        talker("sample_red.mp4")
    except rospy.ROSInterruptException:
        pass