#!/usr/bin/env python3 
# -*- coding: utf-8 -*-
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
from KalmanFilter import KalmanFilter
import numpy as np

class weed_detector_cv:
    def __init__(self):
        self.bridge = CvBridge()
        self.x = 0
        self.y = 0
        
        self.model = YOLO('best.pt') 
          
        self.rgb_sub = rospy.Subscriber("/camera/rgb/image_color", Image, self.rgbCallback)
        self.point_pub = rospy.Publisher("/weed_xy", Int16MultiArray, queue_size= 10)
        self.KF = KalmanFilter(0.1, 1, 1, 1, 0.01, 0.01)
        
    def rgbCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            results = self.model.track(cv_image, persist= True, classes = [0,1,2], conf= 0.2)
            
        except CvBridgeError as e:
            print(e)
        
        # reset coordinates 
        self.x = 0
        self.y = 0
        
        annotated_image = results[0].plot()
        for result in results:
            for detection in result.boxes.data:
                if len(detection) == 7:
                    plant = int(detection[6])  # Class ID is in the 6th position (index 6)
                    if plant == 2:
                        x1, y1, x2, y2 = detection[:4]
                        cx = int((x1 + x2) / 2)
                        cy = int((y1 + y2) / 2)
                        pt = np.array([[cx],[cy]])

                        predicted_state= self.KF.predict()
                        cv2.circle(annotated_image, [int(predicted_state[0, 0]),int(predicted_state[1, 0])], 5, (255,0, 0), 4)
                        estimated_state = self.KF.update(pt) 
                        cv2.circle(annotated_image, [int(estimated_state[0, 0]),int(estimated_state[1, 0])], 7, (0, 0, 255), 4)
                        self.x = int(predicted_state[0, 0])
                        self.y = int(predicted_state[1, 0])
                        cv2.circle(annotated_image, (cx, cy), 10, (0, 255, 0), 2)
                    
        cv2.imshow("Weed Tracking", annotated_image)
        cv2.waitKey(3)
        #cv2.waitKey(3)
        if self.x and self.y != 0:
            point_msg = Int16MultiArray()
            point_msg.data = [self.x, self.y]
            self.point_pub.publish(point_msg)
            
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
            