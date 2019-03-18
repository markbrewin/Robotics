# -*- coding: utf-8 -*-
"""
Created on Mon Mar 18 02:57:10 2019

@author: student
"""

import actionlib
import cv2
import numpy
import rospy

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String

class assignment:
    def __init__(self):
        cv2.startWindowThread()        
        
        self.bridge = CvBridge()
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server() 
        
        self.cam = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callbackImage)
        self.laser = rospy.Subscriber("/scan", LaserScan, self.callbackScan)
        self.log = rospy.Publisher('/log', String, queue_size = 1)
        self.veloc = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        
        self.curWaypoint = 0
        self.distance = 0
        self.found = [False, False, False, False]
        self.twist = Twist() 
        
        self.threshYelLow = numpy.array((20, 100, 100))
        self.threshYelHigh = numpy.array((30, 255, 255))
        self.threshBluLow = numpy.array((110, 50, 50))
        self.threshBluHigh = numpy.array((130, 255, 255))
        self.threshRedLow = numpy.array((0, 100, 100))
        self.threshRedHigh = numpy.array((10, 255, 255))
        self.threshGreLow = numpy.array((50, 100, 100))
        self.threshGreHigh = numpy.array((70, 255, 255))
        
        self.waypoints = [
            [(-0.030, 0.614, 0.0), (0.0, 0.0, 0.935, -0.297), "Center Area"],
            [(2.473, -4.569, 0.0), (0.0, 0.0, 0.982, -0.188), "Corridor Top"],
            [(-2.124, -4.246, 0.0), (0.0, 0.0, 1, -0.026), "Corridor Bottom"],
            [(0.290, -5.080, 0.0), (0.0, 0.0, 0.7, 0.6), "Corridor Center"],
            [(-4.161, 0.316, 0.0), (0.0, 0.0, 0.0, 1.0), "Offices"],
            [(-4.145, 5.149, 0.0), (0.0, 0.0, 0.0, 1.0), "Behind Table"],
            [(1.167, 4.441, 0.0), (0.0, 0.0, -0.032, 0.995), "Cupboards"]
        ]
        
        self.setNextWaypoint()
    
    def setNextWaypoint(self):
        if self.curWaypoint >= len(self.waypoints):
            self.curWaypoint = 0
        
        waypoint = self.waypoints[self.curWaypoint]
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose.position.x = waypoint[0][0]
        goal.target_pose.pose.position.y = waypoint[0][1]
        goal.target_pose.pose.position.z = waypoint[0][2]
        goal.target_pose.pose.orientation.x = waypoint[1][0]
        goal.target_pose.pose.orientation.y = waypoint[1][1]
        goal.target_pose.pose.orientation.z = waypoint[1][2]
        goal.target_pose.pose.orientation.w = waypoint[1][3]
        
        self.log.publish("Moving to Waypoint")        
        
        self.client.send_goal(goal, done_cb = self.waypointReached)    
        
    def waypointReached(self, state, result):
        if state == actionlib.GoalStatus.SUCCEEDED:
            self.curWaypoint += 1
            self.setNextWaypoint()
    
    def callbackImage(self, data):
        cv2.namedWindow("Robot Cam", 1)
        
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e

        imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        maskYellow = cv2.inRange(imgHSV, self.threshYelLow, self.threshYelHigh)
        maskBlue = cv2.inRange(imgHSV, self.threshBluLow, self.threshBluHigh)
        maskRed = cv2.inRange(imgHSV, self.threshRedLow, self.threshRedHigh)
        maskGreen = cv2.inRange(imgHSV, self.threshGreLow, self.threshGreHigh)
        mask = maskYellow + maskBlue + maskRed + maskGreen

        h, w, d = img.shape
        
        #Limits robot's view to objects close by preventing detecting objects in distance.
        search_top = 3*h/4
        search_bot = 3*h/4 + 20
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        
        momentsAll = cv2.moments(mask)
        
        if momentsAll['m00'] > 0:
            for colID in range(4):
                if colID == 0:
                    moments = cv2.moments(maskYellow)
                elif colID == 1:
                    moments = cv2.moments(maskBlue)
                elif colID == 2:
                    moments = cv2.moments(maskRed)
                elif colID == 3:
                    moments = cv2.moments(maskGreen)
                
                if moments['m00'] > 0 and self.found[colID] == False and self.distance > 1:   
                    self.client.cancel_all_goals()
        
                    cx = int(moments['m10']/moments['m00'])
                    cy = int(moments['m01']/moments['m00'])
                    
                    cv2.circle(img, (cx, cy), 20, (0, 0, 255), -1)
                    
                    err = cx - w/2
                    
                    self.twist.linear.x = 0.2
                    self.twist.angular.z = -float(err) / 100
                
                    self.veloc.publish(self.twist)
                    
                    break
                
                elif moments['m00'] > 0 and self.found[colID] == False and self.distance <= 1:
                    self.found[colID] = True
                    self.curWaypoint += 1
                    self.setNextWaypoint()
        
        cv2.imshow("Robot Cam", img)
                    
    def callbackScan(self, data):   
        self.distance = data.ranges[len(data.ranges)/2]

if __name__ == '__main__':    
    rospy.init_node('assignment')
    
    assignment()