#!/usr/bin/env python

import rospy
import cv2
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import numpy as np
import imutils
from collections import deque

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

        rospy.init_node('ball_follow', anonymous=False)

        #subscribers
        rospy.Subscriber('bebop/image_raw', Image, self.image_callback)

        #publishers

    def shutdown(self):
        rospy.loginfo("Shutting down ball_follow node.")
        rospy.sleep()

    # call backs for image
    def image_callback(self, frame):
        # TODO: get frame, find red pixel
        frame = imutils.resize(frame, width=600)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, redLower, redUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # find contours
        # (x, y) cetner of ball
        cnts = cv2.findContour(mask.copy(), cv2.RETR_EXTERNAL, 
            cv2.CHAIN_APPROX_SIMPLE)[-2]
        cetner = None

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
            pts.appendleft(center)

            # loop over tracked pts

    # move/follow node

if __name__ == '__main__':
    try:
        rospy.loginfo("Starting ball_follow node.")
        BallFollow()
        rospy.spin()
    except rospy.ROSInterrupException:
        rospy.logerr("Ball_follow node terminated.")