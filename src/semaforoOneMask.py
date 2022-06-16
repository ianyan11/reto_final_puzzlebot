#!/usr/bin/env python3

#from _future_ import division

import rospy
import cv2 as cv
import numpy as np
import sys
import notCvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist,Vector3 

class trafficLigthDetector():

    def __init__(self):
        rospy.init_node('traffic_ligth')
        rospy.Subscriber('/video_source/raw', Image, self.sempahore_callback)
        self.statePublisher = rospy.Publisher('ligth_state', String, queue_size=1)
        #Mask values
        self.red1Low = np.array([129,70,140], np.uint8)
        self.red1Up = np.array([179,255,255], np.uint8)
        #self.red2Low = np.array([150,60,130], np.uint8)
        #self.red2Up = np.array([179,255,225], np.uint8)
        self.green1Low = np.array([60,60,150], np.uint8)
        self.green1Up = np.array([100,255,255], np.uint8)
        #self.green2Low = np.array([36,52,72], np.uint8)
        #self.green2Up = np.array([86,255,255], np.uint8)
        

    def sempahore_callback(self, data):
        """Callback method for semaphore processing"""
        #Convert cv format
        img = notCvBridge.imgmsg_to_cv2(data)
        #Process image
        img = cv.rotate(img, cv.ROTATE_180) 
        img = cv.resize(img, (int(img.shape[1] * .8),int(img.shape[0] * .8)))
        img = img[0:int(len(img)*.5),0:len(img[0])] #Crop image since we dont need to see nothing on floor
        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        #Generate masks
        maskR1 = cv.inRange(hsv, self.red1Low, self.red1Up)
        maskG1 = cv.inRange(hsv, self.green1Low, self.green1Up)
        #Apply
        resultR = cv.bitwise_and(img, img, mask=maskR1)
        resultG = cv.bitwise_and(img, img, mask=maskG1)
        blurR=cv.medianBlur(resultR,5)
        bgrR=cv.cvtColor(blurR,cv.COLOR_HSV2BGR)
        bgrRsized = cv.resize(bgrR, (int(bgrR.shape[1] * .5),int(bgrR.shape[0] * .5)))
        #Green
        blurG=cv.medianBlur(resultG,3)
        bgrG=cv.cvtColor(blurG,cv.COLOR_HSV2BGR)
        bgrGsized = cv.resize(bgrG, (int(bgrG.shape[1] * .5),int(bgrG.shape[0] * .5)))
        cv.imshow('no maks',img)
        
        cv.imshow('red maks',bgrRsized)
        cv.imshow('green maks',bgrGsized)
        cv.waitKey(1)

def main():
    h = trafficLigthDetector()
    rospy.spin()

if __name__ == "__main__":
    main()