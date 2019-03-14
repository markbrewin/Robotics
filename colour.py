# -*- coding: utf-8 -*-
"""
Created on Mon Feb 25 15:15:44 2019

@author: Mark Brewin
"""

import actionlib
import cv2
import numpy
import rospy

from smach import State, StateMachine
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Image, LaserScan

waypoints = [
    [(-0.030, 0.614, 0.0), (0.0, 0.0, 0.935, -0.297), "Center Area"],
    [(2.473, -4.569, 0.0), (0.0, 0.0, 0.982, -0.188), "Corridor Top"],
    [(-2.124, -4.246, 0.0), (0.0, 0.0, 1, -0.026), "Corridor Bottom"],
    [(0.290, -5.080, 0.0), (0.0, 0.0, 0.7, 0.6), "Corridor Center"],
    [(-4.161, 0.316, 0.0), (0.0, 0.0, 0.0, 1.0), "Offices"],
    [(-4.145, 5.149, 0.0), (0.0, 0.0, 0.0, 1.0), "Behind Table"],
    [(1.167, 4.441, 0.0), (0.0, 0.0, -0.032, 0.995), "Cupboards"]
]

client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

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

class Spin(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        
    def execute(self, userdata):
        print('spin')
        #sleep(1)
        return 'success'
        
class MoveToWaypoint(State):
    def __init__(self, w):
        self.pose = waypoints[w]
        State.__init__(self, outcomes=['success'])
        
    def execute(self, userdata):
        print(self.pose)        
        
        poseGoal = MoveBaseGoal()
        poseGoal.target_pose.header.frame_id = 'map'
        
        poseGoal.target_pose.pose.position.x = self.pose[0][0]
        poseGoal.target_pose.pose.position.y = self.pose[0][1]
        poseGoal.target_pose.pose.position.z = self.pose[0][2]
        
        poseGoal.target_pose.pose.orientation.x = self.pose[1][0]
        poseGoal.target_pose.pose.orientation.y = self.pose[1][1]
        poseGoal.target_pose.pose.orientation.z = self.pose[1][2]
        poseGoal.target_pose.pose.orientation.w = self.pose[1][3]        
        
        client.send_goal(poseGoal)
        client.wait_for_result()
        
        return 'success'

if __name__ == '__main__':
    #colourSearch()
    rospy.init_node('colourSearch', anonymous=True)
    
    client.wait_for_server()
    
    search = StateMachine(outcomes=['success'])
    with search:
        for w in range(len(waypoints)):
            StateMachine.add('MoveToWaypoint' + str(w), MoveToWaypoint(w), transitions={'success': 'SearchForColour' + str(w)})
            
            if w > len(waypoints):
                StateMachine.add('SearchForColour' + str(w), Spin(), transitions={'success': 'MoveToWaypoint' + str(w + 1)})
            else:
                StateMachine.add('SearchForColour' + str(w), Spin(), transitions={'success': 'MoveToWaypoint0'})
                              
        search.execute()
    
    cv2.destroyAllWindows()