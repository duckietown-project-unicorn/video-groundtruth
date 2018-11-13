#!/usr/bin/env python
###############################################################################
# Duckietown - Project Unicorn ETH
# Author: Simon Schaefer
# Python ROS node to estimate robot's pose based on color filtering the input
# image (robot tagged with scene-unique color strip) and exploiting the known
# intersection structure. 
###############################################################################
import cv2
from cv_bridge import CvBridge, CvBridgeError
from os.path import isfile
import numpy as np
import rospy
from sensor_msgs.msg import Image



class Main(): 

    def __init__(self): 
        self.bridge = CvBridge()
        rospy.Subscriber("/external/img", Image, self.callback)
        self.mask_pub = rospy.Publisher('external/masked', Image, queue_size=10)
        rospy.spin()
        
    def callback(self, data):
        """Convert image data to numpy data, filter out unique color (robot)
        and determine its position in the image."""
        try: 
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logfatal(e)
        # Mask color in unique-color range (as HSV color since color itself
        # is light intensity independent, in opposite to RGB). 
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)   
        lower_red = np.array([162,100,100])
        upper_red = np.array([164,255,200])
        mask = cv2.inRange(hsv, lower_red, upper_red)
        res = cv2.bitwise_and(frame,frame, mask=mask)
        # find contours in the thresholded image and initialize the
        # shape detector
        _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
            if len(approx) > 15: 
                cv2.drawContours(res,[cnt],0,255,10)
                print(len(cnt))
        # Publish resulting mask. 
        try:
            self.mask_pub.publish(self.bridge.cv2_to_imgmsg(res, "bgr8"))
        except CvBridgeError as e:
            rospy.logfatal(e)

if __name__ == '__main__':
    rospy.init_node('external_pose_estimator', anonymous=True)
    try:
        Main()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
