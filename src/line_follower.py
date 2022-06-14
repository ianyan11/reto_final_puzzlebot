#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from notCvBridge import imgmsg_to_cv2, cv2_to_imgmsg
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Empty
from imutils.perspective import four_point_transform

class Line_Follower():
    def __init__(self) -> None:
        rospy.Subscriber('/video_source/raw', Image, self.process_image)
        self.center_line_pub = rospy.Publisher('/line_pose', Float32, queue_size=1)
        self.horizontal_line_pub = rospy.Publisher('/horizontal_line', Empty, queue_size=1)
        self.image_pub = rospy.Publisher('/video_edites', Image, queue_size=1)

        self.tolerance = 22000
        #self.twist = Twist()
    
    def process_image(self, msg: Image) ->None:
    
        #pasa de imagen a formato cv2
        image = imgmsg_to_cv2(msg)
        #rotate image
        image = cv2.rotate(image, cv2.ROTATE_180)
        
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) 

        self.center_line(gray)
        self.horizontal_line(image)

    def center_line(self, img: np.array) -> None:
        #reduce image siz
        img = cv2.resize(img, (int(img.shape[1] * .2),int(img.shape[0] * .25)))
        img = four_point_transform(img, np.array([[60,140],[len(img[0])-60,140],[len(img[0]),len(img)],[0, len(img)]]))

        #gaussian blur
        img = cv2.GaussianBlur(img, (3,3), 0)
        #Dilate and Erode
        img = cv2.dilate(img, None, iterations=3)
        img = cv2.erode(img, None, iterations=3)
        
        s = np.sum(img, axis=0)
        #get index of smallest value
        min_index = np.argmin(s)
        half =int(len(s)/2)+1
        self.center_line_pub.publish((min_index-half)/half)
    
    def horizontal_line(self, img: np.array) -> None:
        #reduce image siz
        img = cv2.resize(img, (int(img.shape[1] * .2),int(img.shape[0] * .25)))
        #gaussian blur
        img = cv2.GaussianBlur(img, (3,3), 0)
        #Dilate and Erode
        img = cv2.dilate(img, None, iterations=2)
        img = cv2.erode(img, None, iterations=1)
        #Get V in hsv
        data = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        _,_,v = cv2.split(data)
        #self.image_pub.publish(cv2_to_imgmsg(img))
        v = np.sum(img, axis=1)
        #get index of smallest value
        min = np.min(v)
        rospy.loginfo(min)
        if(min < self.tolerance):
            self.horizontal_line_pub.publish(Empty())





def main():
    rospy.init_node('line_follower', anonymous=True)
    l = Line_Follower()
    rospy.spin()


if __name__ == "__main__":
    main()
    
