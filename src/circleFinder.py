#!/usr/bin/env python3

import cv2
from cv2 import imshow
import rospy
from sensor_msgs.msg import Image
from notCvBridge import imgmsg_to_cv2, cv2_to_imgmsg
import numpy as np

class circleFinder:

    def __init__(self):
        self.detectionPublisher = rospy.Publisher('/cropped_image', Image, queue_size=1)
        rospy.Subscriber('/video_source/raw', Image, self.imageCallback)
        
    def imageCallback(self, msg):
        image = cv2.resize(imgmsg_to_cv2(msg),(0,0),fx=0.3,fy=0.3)
        image = cv2.rotate(image, cv2.ROTATE_180)
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        #Gaussian blur
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        
        # Find circles hough gradient alt
        circles = cv2.HoughCircles(blur, cv2.HOUGH_GRADIENT_ALT,20, 30, param1=300,
                            param2=0.85, minRadius=20)
        # Draw circles
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                try:
                    crop_img = image[i[1]-i[2]:i[1]+i[2], i[0]-i[2]:i[0]+i[2]]
                    self.detectionPublisher.publish((cv2_to_imgmsg(crop_img)))
                    blur = cv2.circle(blur,(i[0],i[1]),i[2],(0,255,0),2)          
                except:
                    None 

            imshow('edges', blur)
            cv2.waitKey(1)

           
            
      

if __name__ == '__main__':
    rospy.init_node('circle_finder', anonymous=True)
    c = circleFinder()
    rospy.spin()
