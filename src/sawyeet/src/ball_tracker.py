#!/usr/bin/env python
# import rospy
# from sensor_msgs import Image
# from cv_bridge import CvBridge
import cv2
import imutils
import numpy as np

# def callback(data):
#     # bridge = CvBridge()
#     frame = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
#     cv2.imshow("frame1", frame)

def get_centroid(frame):
    result = frame.copy()
    frame = cv2.GaussianBlur(frame, (15, 15), 0)
    image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
 
    # lower boundary RED color range values; Hue (0 - 10)
    lower1 = np.array([0, 100, 20])
    upper1 = np.array([10, 255, 255])
    
    # upper boundary RED color range values; Hue (160 - 180)
    lower2 = np.array([160,100,20])
    upper2 = np.array([179,255,255])

    # lower1 = (20, 80, 20)
    # upper1 = (40, 255, 255)
    
    lower_mask = cv2.inRange(image, lower1, upper1)
    upper_mask = cv2.inRange(image, lower2, upper2)
    
    full_mask = lower_mask + upper_mask
    full_mask = cv2.erode(full_mask, None, iterations=2)
    full_mask = cv2.dilate(full_mask, None, iterations=2)
    full_mask = cv2.morphologyEx(full_mask, cv2.MORPH_OPEN, np.ones((30, 30), np.uint8))
    
    result = cv2.bitwise_and(result, result, mask=full_mask)


    # find contours in the mask and initialize the current
	# (x, y) center of the ball
    cnts = cv2.findContours(full_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None
    
	# only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
    else:
        radius = 0
        x = 0
        y = 0

    return center, radius, x, y

def listener():
    # rospy.init_node('ball_tracker', anonymous=True)
    # rospy.Subscriber("image_raw", Image, callback)
    

    # rospy.spin()
    cap = cv2.VideoCapture('sample.mp4')

    # Check if camera opened successfully
    if (cap.isOpened()== False): 
        print("Error opening video stream or file")

    # Read until video is completed
    pts = []
    while(cap.isOpened()):
        # Capture frame-by-frame
        ret, frame = cap.read()
        if ret == True:
            # Display the resulting frame
            # frame = get_centroid(frame)
            center, radius, x, y = get_centroid(frame)
            if radius > 10:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
                cv2.circle(frame, (int(x), int(y)), int(radius),
                    (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
            pts.append(center)

            # loop over the set of tracked points
            for i in range(1, len(pts)):
                if pts[i - 1] is None or pts[i] is None:
                    continue
  
                thickness = int(np.sqrt(200 / float(i + 1)) * 2.5)
                cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)
                
            cv2.imshow('Frame',frame)


            # Press Q on keyboard to  exit
            if cv2.waitKey(25) & 0xFF == ord('q'):
                break

        # Break the loop
        else: 
            break


if __name__ == '__main__':
    listener()