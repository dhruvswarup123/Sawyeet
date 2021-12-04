#!/usr/bin/env python
# import rospy
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
from tracking_utils_copy import *
import cv2


def talker(video_file):
    # https://www.myzhar.com/blog/tutorials/tutorial-opencv-ball-tracker-using-kalman-filter/
    # pub = rospy.Publisher('image_raw', Image, queue_size=10)
    # rospy.init_node('usb_cam_temp', anonymous=True)
    # rate = rospy.Rate(15)

    cap = cv2.VideoCapture(video_file)
    # Check if camera opened successfully
    if (cap.isOpened()== False): 
        print("Error opening video stream or file")

    # Kalman filter stuff
    stateSize = 6
    measSize = 4
    contrSize = 0
    kf = cv2.KalmanFilter(stateSize, measSize, contrSize)
    state = np.array((stateSize, 1), np.float32)
    meas = np.array((measSize, 1), np.float32)

    kf.transitionMatrix = np.identity(kf.transitionMatrix.shape[0])

    kf.measurementMatrix = np.zeros((measSize, stateSize), np.float32)
    kf.measurementMatrix[0, 0] = 1
    kf.measurementMatrix[1, 1] = 1
    kf.measurementMatrix[2, 4] = 1
    kf.measurementMatrix[3, 5] = 1

    kf.processNoiseCov = np.identity(kf.processNoiseCov.shape[0]) * 1e-2
    kf.processNoiseCov[0, 0] = 1e-2
    kf.processNoiseCov[1, 1] = 1e-2
    kf.processNoiseCov[2, 2] = 5
    kf.processNoiseCov[3, 3] = 5
    kf.processNoiseCov[4, 4] = 1e-2
    kf.processNoiseCov[5, 5] = 1e-2

    kf.measurementNoiseCov = np.identity(kf.measurementNoiseCov.shape[0]) * 1e-1

    ticks = 0
    found = False

    while(cap.isOpened()):
        precTick = ticks
        ticks = cv2.getTickCount()
        dT = (ticks - precTick) / cv2.getTickFrequency() 

        ret, frame = cap.read()

        if ret == True:
            
            # center, _, mask = get_centroid(frame, 'tennis')
            if found:
                kf.transitionMatrix[]






            result = frame.copy()

            # Blur the frame and convert to hsv color space
            # frame = cv2.GaussianBlur(frame, (15, 15), 0)
            frame = cv2.medianBlur(frame, 5)
            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Get mask that removes pixels that are not that color
            mask = get_mask(hsv_frame, color)
            
            # Apply mask
            result = cv2.bitwise_and(result, result, mask=mask)

            # find contours in the mask and initialize the current
            # (x, y) center of the ball
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            center = None
            radius = 0
            c = []
            
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

                
                    if cv2.waitKey(25) & 0xFF == ord('q'):
                        break

                # Break the loop
                else: 
                    break


if __name__ == '__main__':
    # try:
    talker("random/sample_720_tennis.mp4")
    # except rospy.ROSInterruptException:
    #     pass