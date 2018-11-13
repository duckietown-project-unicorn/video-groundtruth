#!/usr/bin/env python
###############################################################################
# Duckietown - Project Unicorn ETH
# Author: Simon Schaefer
# Python ROS node to read a video, split it in images and publish images 
# together with timestamp as sensor_msgs/Img message. 
###############################################################################
import cv2
from cv_bridge import CvBridge, CvBridgeError
from os.path import isfile
import rospy
from sensor_msgs.msg import Image

def main():
    pub = rospy.Publisher('external/img', Image, queue_size=10)
    rate = rospy.Rate(10) 
    video_path = str(rospy.get_param("/video_img_pub/video_path_arg"))
    if not isfile(video_path): 
        rospy.logfatal("Video path launch argument not set or not found!") 
        exit(0)
    cap = cv2.VideoCapture(video_path)
    bridge = CvBridge()
    rospy.loginfo("Starting external video output")
    while not rospy.is_shutdown():
        success, frame = cap.read()
        if success: 
            try:
                pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
            except CvBridgeError as e:
                rospy.logfatal(e)
        else: 
            break
        rate.sleep()
    rospy.loginfo("Ending external video output")
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('external_video_pub', anonymous=True)
    try:
        main()
    except rospy.ROSInterruptException:
        pass

