# -*- coding: utf-8 -*-
"""
Created on Mon Feb 25 15:15:44 2019

@author: Mark Brewin
"""

import rospy
import cv2
import numpy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError


class colour:

    def __init__(self):

        cv2.startWindowThread()
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",
                                          Image, self.callback)
                                          
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        self.twist = Twist()

    def callback(self, data):

        cv2.namedWindow("Image window", 1)
        
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e

        imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        threshYelLow = numpy.array((20, 100, 100))
        threshYelHigh = numpy.array((30, 255, 255))
        
        threshBluLow = numpy.array((110, 50, 50))
        threshBluHigh = numpy.array((130, 255, 255))
        
        threshRedLow = numpy.array((0, 100, 100))
        threshRedHigh = numpy.array((10, 255, 255))
        
        threshGreLow = numpy.array((36, 25, 25))
        threshGreHigh = numpy.array((70, 255, 255))
        
        maskYellow = cv2.inRange(imgHSV, threshYelLow, threshYelHigh)
        maskBlue = cv2.inRange(imgHSV, threshBluLow, threshBluHigh)
        maskRed = cv2.inRange(imgHSV, threshRedLow, threshRedHigh)
        maskGreen = cv2.inRange(imgHSV, threshGreLow, threshGreHigh)

        h, w, d = img.shape
        
        momentsAll = cv2.moments(maskYellow + maskBlue + maskRed + maskGreen)
        
        if momentsAll['m00'] > 0:
            
            for colID in range(4):
                if colID == 0:
                    moments = cv2.moments(maskYellow)
                    colour = "Yellow"
                elif colID == 1:
                    moments = cv2.moments(maskBlue)
                    colour = "Blue"
                elif colID == 2:
                    moments = cv2.moments(maskRed)
                    colour = "Red"
                elif colID == 3:
                    moments = cv2.moments(maskGreen)
                    colour = "Green"
                
                if moments['m00'] > 0:
                    break
            
            print(colour + " found. (" + str(colID) + ")")            
            
            cx = int(moments['m10']/moments['m00'])
            cy = int(moments['m01']/moments['m00'])
            
            cv2.circle(img, (cx, cy), 20, (0, 0, 255), -1)
            
            err = cx - w/2
            
            self.twist.linear.x = 0.2
            self.twist.angular.z = -float(err) / 100
        
            print self.twist.angular.z

            self.cmd_vel_pub.publish(self.twist)
            
        cv2.imshow("Image window", img)
        cv2.waitKey(1)


colour()
rospy.init_node('colour', anonymous=True)
rospy.spin()
cv2.destroyAllWindows()