#!/usr/bin/env python3 
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class weed_detector_cv:
    def __init__(self):
        self.bridge = CvBridge()
        self.x = 0
        self.y = 0
        self.z = 0 
        
        self.rgb_sub = rospy.Subscriber("/camera/rgb/image_color", Image, self.rgbCallback)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depthCallback)
        self.point_pub = rospy.Publisher("/weed_xy", Int16MultiArray, queue_size= 10)
        
    def rgbCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        # reset coordinates 
        self.x = 0
        self.y = 0
        
        cv_image = self.get_keypoints(cv_image)
        
        cv2.imshow("Image window", cv_image)
        #rospy.loginfo("x: %.2f, y: %.2f, z: %.2f", self.x, self.y, self.z)
        cv2.waitKey(3)
        if self.x and self.y != 0:
            point_msg = Int16MultiArray()
            point_msg.data = [self.x, self.y]
            self.point_pub.publish(point_msg)
            
        
    def depthCallback(self, data):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(data,"16UC1")
        except CvBridgeError as e:
                print(e)
        self.z = depth_image[self.y, self.x] 
        
    def get_keypoints(self, img):
        # Convert the image to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur to the grayscale image
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        circles = cv2.HoughCircles(blurred,cv2.HOUGH_GRADIENT,1,20, param1=70,param2=30,minRadius=20,maxRadius=45)
        
        if circles is not None:
            
            circles = np.uint16(np.around(circles))
            
            for i in circles[0,:]:
                # draw the outer circle
                cv2.circle(img,(i[0],i[1]),i[2],(0,255,0),2)
                # draw the center of the circle
                cv2.circle(img,(i[0],i[1]),2,(0,0,255),3)
                
                self.x = (i[0])
                self.y = (i[1])
            cv2.putText(img, f'z: {self.z:.2f}',(self.x, self.y),cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,255,0), 2)
        
        return img
            
#-----------------------------------------------------------------------------------------------------------------          

def main(args):
    try: 
        rospy.init_node('weed_detector_cv', anonymous= True)
        weed_pose = weed_detector_cv()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
            