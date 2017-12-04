#!/usr/bin/env python

import rospy
import cv2
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import numpy as np
import imutils
from collections import deque
from cv_bridge import CvBridge, CvBridgeError

# color boundary
redLower = (108, 48, 0)
redUpper = (255, 0, 0)
pts_buffer = 10

class BallFollow():
    def __init__(self):
        self.ball_x = 0
        self.ball_y = 0

        self.center_x = 0
        self.center_y = 0
        self.pts = deque(maxlen=pts_buffer)

        self.bridge = CvBridge()

        rospy.init_node('ball_follow', anonymous=False)

        #subscribers
        rospy.Subscriber('bebop/image_raw', Image, self.image_callback)

        #publishers

    def shutdown(self):
        rospy.loginfo("Shutting down ball_follow node.")
        rospy.sleep()

    # call backs for image
    def image_callback(self, image):
        # TODO: get frame, find red pixel
        frame = self.bridge.imgmsg_to_cv2(image, "bgr8")
        frame = imutils.resize(frame, width=600)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, redLower, redUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # find contours
        # (x, y) cetner of ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, 
            cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None

        if len(cnts) > 0:
            # find the largest contour on the mask and use it to calc circle
            c = max(cnts, key=cv2.contourArea)
            ((self.ball_x, self.ball_y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # minimum radius
            if radius > 10:
                # draw circle and center
                # update tracked pts
                cv2.circle(frame, (int(self.ball_x), int(self.ball_y)), int(radius),
                    (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)

            # update the pts queue
        self.pts.appendleft(center)

        # loop over tracked pts
        for i in xrange(1, len(self.pts)):
            # if the points are None, ignore
            if self.pts[i - 1] is None or self.pts[i] is None:
                continue

            # otherwise, compute thickness of the line, draw connecting line
            thickness = int(np.sqrt(pts_buffer / float(i + 1)) * 2.5)
            cv2.line(frame, self.pts[i - 1], self.pts[i], (0, 0, 255), thickness)

        # show the frame to the screen
        cv2.imshow("Frame", frame)
        cv2.waitKey(3)

        # if the 'q' key is pressed, stop the loop
        #if key == ord("q"):
            #break



    # move/follow node

if __name__ == '__main__':
    try:
        rospy.loginfo("Starting ball_follow node.")
        BallFollow()
        rospy.spin()
    except rospy.ROSInterrupException:
        rospy.logerr("Ball_follow node terminated.")