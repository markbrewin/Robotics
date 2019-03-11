# -*- coding: utf-8 -*-
"""
Created on Mon Feb 25 15:15:44 2019

@author: Mark Brewin
"""

import actionlib
import cv2
import numpy
import rospy

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Image, LaserScan

waypoints = [
    [(0.290, -5.080, 0.0), (0.0, 0.0, 0.7, 0.6)],
    [(-4.145, 5.149, 0.0), (0.0, 0.0, 0.0, 1.0)],
    [(-4.161, 0.316, 0.0), (0.0, 0.0, 0.0, 1.0)]
]

class colourSearch:

    def __init__(self):
        cv2.startWindowThread()
        self.bridge = CvBridge()
        
        self.subImage = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callbackImage)
        self.subScan = rospy.Subscriber("/scan", LaserScan, self.callbackScan)
                                          
        self.pubCmdVel = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        
        self.twist = Twist()
        
        self.distance = 0        
        self.found = [False, False, False, False]
        
        self.threshYelLow = numpy.array((20, 100, 100))
        self.threshYelHigh = numpy.array((30, 255, 255))
        
        self.threshBluLow = numpy.array((110, 50, 50))
        self.threshBluHigh = numpy.array((130, 255, 255))
        
        self.threshRedLow = numpy.array((0, 100, 100))
        self.threshRedHigh = numpy.array((10, 255, 255))
        
        self.threshGreLow = numpy.array((50, 100, 100))
        self.threshGreHigh = numpy.array((70, 255, 255))

    def callbackImage(self, data):
        cv2.namedWindow("Image window", 1)
        
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e

        imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        maskYellow = cv2.inRange(imgHSV, self.threshYelLow, self.threshYelHigh)
        maskBlue = cv2.inRange(imgHSV, self.threshBluLow, self.threshBluHigh)
        maskRed = cv2.inRange(imgHSV, self.threshRedLow, self.threshRedHigh)
        maskGreen = cv2.inRange(imgHSV, self.threshGreLow, self.threshGreHigh)

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
                
                if moments['m00'] > 0 and self.found[colID] == False and self.distance > 1:                           
                    cx = int(moments['m10']/moments['m00'])
                    cy = int(moments['m01']/moments['m00'])
                    
                    cv2.circle(img, (cx, cy), 20, (0, 0, 255), -1)
                    
                    err = cx - w/2
                    
                    self.twist.linear.x = 0.2
                    self.twist.angular.z = -float(err) / 100
                
                    self.pubCmdVel.publish(self.twist)
                    
                    break
                
                elif moments['m00'] > 0 and self.found[colID] == False and self.distance <= 1:
                    print(colour + " found. (" + str(colID) + ")")
                    self.found[colID] = True
                    print self.found
        
                    break
            
        cv2.imshow("Image window", img)
        cv2.waitKey(1)
        
    def callbackScan(self, data):   
        self.distance = data.ranges[len(data.ranges)/2]        
        print("Distance: " + str(self.distance))
        
def poseGoal(pose):
    poseGoal = MoveBaseGoal()
    poseGoal.target_pose.header.frame_id = 'map'
    poseGoal.target_pose.pose.position.x = pose[0][0]
    poseGoal.target_pose.pose.position.y = pose[0][1]
    poseGoal.target_pose.pose.position.z = pose[0][2]
    poseGoal.target_pose.pose.orientation.x = pose[1][0]
    poseGoal.target_pose.pose.orientation.y = pose[1][1]
    poseGoal.target_pose.pose.orientation.z = pose[1][2]
    poseGoal.target_pose.pose.orientation.w = pose[1][3]
    
    return poseGoal

if __name__ == '__main__':
    #colourSearch()
    rospy.init_node('colourSearch', anonymous=True)
    
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
        
    while True:
        for pose in waypoints:
            goal = poseGoal(pose)
            client.send_goal(goal)
            client.wait_for_result()
    
    cv2.destroyAllWindows()