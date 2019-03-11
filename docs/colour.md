```
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
        mask = cv2.inRange(imgHSV, threshYelLow, threshYelHigh)

        h, w, d = img.shape
        
        M = cv2.moments(mask)
        
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            
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
```
