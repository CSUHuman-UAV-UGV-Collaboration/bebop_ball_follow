#!/usr/bin/env python

import rospy
import cv2
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import numpy as np

# color boundary
redLower = (108, 48, 0)
redUpper = (255, 0, 0)

class BallFollow():
    def __init__(self):
        self.ball_x = 0
        self.ball_y = 0

        self.center_x = 0
        self.center_y = 0

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

    # move/follow node

if __name__ == '__main__':
    try:
        rospy.loginfo("Starting ball_follow node.")
        BallFollow()
        rospy.spin()
    except rospy.ROSInterrupException:
        rospy.logerr("Ball_follow node terminated.")