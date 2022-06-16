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
        self.red1Low = np.array([0,60,130], np.uint8)
        self.red1Up = np.array([9,255,255], np.uint8)
        self.red2Low = np.array([160,60,130], np.uint8)
        self.red2Up = np.array([179,255,225], np.uint8)
        self.green1Low = np.array([60,60,150], np.uint8)
        self.green1Up = np.array([100,255,255], np.uint8)
        self.green2Low = np.array([36,52,72], np.uint8)
        self.green2Up = np.array([86,255,255], np.uint8)
        

    def sempahore_callback(self, data):
        """Callback method for semaphore processing"""
        #Convert cv format
        img = notCvBridge.imgmsg_to_cv2(data)
        #Process image
        img = cv.rotate(img, cv.ROTATE_180) 
        img = cv.resize(img, (int(img.shape[1] * .8),int(img.shape[0] * .8)))
        img = img[0:int(len(img)*.28),0:len(img[0])] #Crop image since we dont need to see nothing on floor
        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        #Generate masks
        maskR1 = cv.inRange(hsv, self.red1Low, self.red1Up)
        maskR2 = cv.inRange(hsv, self.red2Low, self.red2Up)
        maskG1 = cv.inRange(hsv, self.green1Low, self.green1Up)
        maskG2 = cv.inRange(hsv, self.green2Low, self.green2Up)
        finalMaskR = cv.add(maskR1, maskR2)
        finalMaskG = cv.add(maskG1, maskG2)
        #Apply
        resultR = cv.bitwise_and(img, img, mask=finalMaskR)
        resultG = cv.bitwise_and(img, img, mask=finalMaskG)
        blurR=cv.medianBlur(resultR,5)
        bgrR=cv.cvtColor(blurR,cv.COLOR_HSV2BGR)
        grayR=cv.cvtColor(bgrR,cv.COLOR_BGR2GRAY)
        treshedR = cv.threshold(grayR,110,255,cv.THRESH_BINARY)[1]
        Rsized = cv.resize(treshedR, (int(treshedR.shape[1] * .5),int(treshedR.shape[0] * .5)))
        #Green
        blurG=cv.medianBlur(resultG,3)
        bgrG=cv.cvtColor(blurG,cv.COLOR_HSV2BGR)
        grayG=cv.cvtColor(bgrG,cv.COLOR_BGR2GRAY)
        treshedG = cv.threshold(grayG,55,255,cv.THRESH_BINARY)[1]
        Gsized = cv.resize(treshedG, (int(treshedG.shape[1] * .5),int(treshedG.shape[0] * .5)))
        #cv.imshow('no maks',img)
        #cv.imshow('red maks',Rsized)
        #cv.imshow('green maks',Gsized)
        #cv.waitKey(1)
        cntsR, hR = cv.findContours(treshedR, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
        cntsG, hG = cv.findContours(treshedG, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

        areaR = 0
        areaG = 0

        for c in cntsR:
            areaR += cv.contourArea(c)
        
        for c in cntsG:
            areaG += cv.contourArea(c)
        
        msg = String()

        if( areaR > areaG):
            msg.data = "RED"
        else:
            msg.data = "GREEN"
        
        self.statePublisher.publish(msg)
            

        

def main():
    h = trafficLigthDetector()
    rospy.spin()

if __name__ == "__main__":
    main()



