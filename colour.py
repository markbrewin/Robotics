# -*- coding: utf-8 -*-
"""
Created on Mon Feb 25 15:15:44 2019

@author: Mark Brewin
"""

import actionlib
import cv2
import numpy
import rospy
import smach_ros

from smach import State, StateMachine
from smach_ros import MonitorState
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Image, LaserScan

found = [False, False, False, False]

threshYelLow = numpy.array((20, 100, 100))
threshYelHigh = numpy.array((30, 255, 255))

threshBluLow = numpy.array((110, 50, 50))
threshBluHigh = numpy.array((130, 255, 255))

threshRedLow = numpy.array((0, 100, 100))
threshRedHigh = numpy.array((10, 255, 255))

threshGreLow = numpy.array((50, 100, 100))
threshGreHigh = numpy.array((70, 255, 255))

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
bridge = CvBridge()

def ColourCheck(ud, data):
    try:
        img = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
        print e

    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
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
            elif colID == 1:
                moments = cv2.moments(maskBlue)
            elif colID == 2:
                moments = cv2.moments(maskRed)
            elif colID == 3:
                moments = cv2.moments(maskGreen)
            
            if moments['m00'] > 0 and found[colID] == False:   
                return True
            
            else:
                return False

class MoveToColour(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        
    def execute(self, userdata):
        cv2.startWindowThread()        
                
        self.distance = 0;
        
        self.subImage = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callbackImage)
        self.subScan = rospy.Subscriber("/scan", LaserScan, self.callbackScan)
        
        self.pubCmdVel = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        
        self.twist = Twist()        
        
    def callbackImage(self, data):
        cv2.namedWindow("Image window", 1)
        
        try:
            img = bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e

        imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
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
                
                if moments['m00'] > 0 and found[colID] == False and self.distance > 1:                           
                    cx = int(moments['m10']/moments['m00'])
                    cy = int(moments['m01']/moments['m00'])
                    
                    cv2.circle(img, (cx, cy), 20, (0, 0, 255), -1)
                    
                    err = cx - w/2
                    
                    self.twist.linear.x = 0.2
                    self.twist.angular.z = -float(err) / 100
                
                    self.pubCmdVel.publish(self.twist)
                    
                    break
                
                elif moments['m00'] > 0 and found[colID] == False and self.distance <= 1:
                    print(colour + " found. (" + str(colID) + ")")
                    found[colID] = True
                    print found
                    
                    return 'success'
            
        cv2.imshow("Image window", img)
        cv2.waitKey(1)
        
    def callbackScan(self, data):   
        self.distance = data.ranges[len(data.ranges)/2]

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
        
class Spin(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        
    def execute(self, userdata):
        print('spin')
        #sleep(1)
        return 'success'

if __name__ == '__main__':
    rospy.init_node('colourSearch', anonymous=True)
    
    client.wait_for_server()   
    
    searchState = StateMachine(outcomes=['success'])
    with searchState:        
        for w in range(len(waypoints)):
            StateMachine.add('MoveToWaypoint' + str(w), 
                         MonitorState("/camera/rgb/image_raw", Image, ColourCheck),
                         transitions = {'invalid': 'Waypoint' + str(w),
                                        'valid': 'MoveToColour',
                                        'preempted': 'Waypoint' + str(w)})            
            
            StateMachine.add('Waypoint' + str(w), MoveToWaypoint(w),
                             transitions={'success': 'SearchForColour' + str(w)})
            
            if w < len(waypoints) - 1:
                StateMachine.add('SearchForColour' + str(w), Spin(),
                                 transitions={'success': 'MoveToWaypoint' + str(w + 1)})
            else:
                StateMachine.add('SearchForColour' + str(w), Spin(),
                                 transitions={'success': 'MoveToWaypoint0'})
                     
        StateMachine.add('MoveToColour', MoveToColour(), transitions={'success': 'MoveToWaypoint0'})       
    
    sis = smach_ros.IntrospectionServer('server_name', searchState, '/SM_ROOT')
    sis.start()    
    searchState.execute()
    
    cv2.destroyAllWindows()