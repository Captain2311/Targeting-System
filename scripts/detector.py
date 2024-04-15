#!/usr/bin/env python3 

import rospy 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 

def webcam_cb(data):
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(data, desired_encoding= "bgr8")
    cv2.imshow("Webcam Stream", img)
    cv2.waitKey(1)

def webcam_subscriber():
    rospy.init_node('webcam_subscriber', anonymous=True)
    rospy.Subscriber('webcam_image', Image, webcam_cb)
    rospy.spin()

if __name__ == '__main__':
    try:
        webcam_subscriber()
    except rospy.ROSInterruptException:
        pass
