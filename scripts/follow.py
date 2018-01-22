#!/usr/bin/env python

import rospy
import cv2
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import numpy as np
import imutils
from collections import deque
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Empty

# color boundary
redLower = (0, 100, 100)
redUpper = (10, 255, 255)
pts_buffer = 10

class BallFollow():
    def __init__(self):
        self.speed = 0.25

        self.ball_x = 0
        self.ball_y = 0
        self.ball_found = False

        self.center_x = 300
        self.center_y = 0
        self.center_zone = 40
        self.pts = deque(maxlen=pts_buffer)

        self.bridge = CvBridge()

        rospy.init_node('ball_follow', anonymous=False)

        #subscribers
        rospy.Subscriber('bebop/image_raw', Image, self.image_callback)

        #publishers
        
        self.pub_takeoff = rospy.Publisher('bebop/takeoff', Empty, queue_size=1, latch=True)
        self.pub_land = rospy.Publisher('bebop/land', Empty, queue_size=1, latch=True)
        self.pub_cmd_vel = rospy.Publisher('bebop/cmd_vel', Twist, queue_size=1)

        takeoff = Empty()
        self.pub_takeoff.publish(takeoff)

    def shutdown(self):
        rospy.loginfo("Shutting down ball_follow node.")
        land = Empty()
        self.pub_land.publish(land)
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
                self.ball_found = True
                cv2.circle(frame, (int(self.ball_x), int(self.ball_y)), int(radius),
                    (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
            else:
                self.ball_found = False
        else:
            self.ball_found = False

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

        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.z = 0
        if self.ball_found:
            if self.ball_x < (self.center_x - self.center_zone):
                vel_msg.angular.z = self.speed
                cv2.putText(frame, "ACTION: turn left", (20,40), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255,0,255))
            elif self.ball_x > (self.center_x + self.center_zone):
                vel_msg.angular.z = -self.speed
                cv2.putText(frame, "ACTION: turn right", (20,40), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255,0,255))
            else:
                vel_msg.angular.z = 0
                cv2.putText(frame, "ACTION: tracked", (20,40), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255,0,255))
        else:
            vel_msg.angular.z = 0
            cv2.putText(frame, "ACTION: ball not found", (20,40), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255,0,255))

        self.pub_cmd_vel.publish(vel_msg)
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