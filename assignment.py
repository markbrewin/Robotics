# -*- coding: utf-8 -*-
"""
Created on Mon Mar 18 02:57:10 2019

@author: Mark Brewin


References:
Quigley, M., Gerkey, B. and Smart, W. (2015). Programming Robots with ROS. 1st ed. Sebastopol: O'Reilly.
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
        #Connects the OpenCV library to allow for image processing.
        self.bridge = CvBridge()
        
        #Action Client allowing for each waypoint pose to be set as a goal for
        #the robot to move towards.
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server() 
        
        #Subscribes to the camera topic and calls the relevant callback
        #to detect the coloured objects.
        self.cam = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callbackImage)
        #Subscribes to the laser scanner topic and calls the relevant callback
        #to find the distance away from objects ahead.
        self.laser = rospy.Subscriber("/scan", LaserScan, self.callbackScan)
        
        #Creates the publisher for the robot's log.
        self.log = rospy.Publisher('/log', String, queue_size = 1)
        #Creates the publisher to send Twist commands to the robot when moving towards a target.
        self.veloc = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)   
        
        #Stores the current distance.
        self.distance = 0
        #Used to create the twist command.
        self.twist = Twist() 
        
        #Keeps track of the current waypoint the robot is heading towards.
        self.curWaypoint = 0
        #Determines whether the robot is currently looking for the coloured objects.
        self.targetSearch = True
        
        #The coloured targets, whether they have been located and their HSV thresholds.
        self.targets = [["Yellow", False, numpy.array((20, 100, 100)), numpy.array((30, 255, 255))],
                        ["Blue", False, numpy.array((90, 200, 30)), numpy.array((120, 255, 230))],
                        ["Red", False, numpy.array((0, 200, 30)), numpy.array((5, 255, 150))],
                        ["Green", False, numpy.array((50, 100, 100)), numpy.array((70, 255, 255))]]
                        
        #Poses for each waypoint of the patrol detailing the position and orientation.
        self.waypoints = [[(-0.030, 0.614, 0.0), (0.0, 0.0, 0.935, -0.297), "Centre (Start)"],
                          [(2.473, -4.569, 0.0), (0.0, 0.0, 0.982, -0.188), "Corridor Top"],
                          [(-2.124, -4.246, 0.0), (0.0, 0.0, 1, -0.026), "Corridor Bottom"],
                          [(0.290, -4.569, 0.0), (0.0, 0.0, 0.7, 0.6), "Corridor Center"],
                          [(1.581, -2.136, 0.0), (0.0, 0.0, 0.320, 0.947), "Corridor Exit"],
                          [(-4.161, 0.316, 0.0), (0.0, 0.0, 0.0, 1.0), "Room 1"],
                          [(-3.701, 2.762, 0.0), (0.0, 0.0, -0.230, 0.973), "Room 2"],
                          [(-4.145, 5.149, 0.0), (0.0, 0.0, 0.0, 1.0), "Behind Table"],
                          [(-3.994, -1.778, 0.0), (0.0, 0.0, 0.011, 0.918), "Room Exit"],
                          [(3.026, 4.441, 0.0), (0.0, 0.0, -0.032, 0.995), "Cupboards"],
                          [(1.047, 4.553, 0.0), (0.0, 0.0, 0.988, 0.153), "Infront of Table"],
                          [(2.816, 2.307, 0.0), (0.0, 0.0, -0.-0.642, 0.766), "Centre (Back)"]]
        
        rospy.sleep(3)
        #Called to set the first waypoint of the robot.
        self.setNextWaypoint()
    
    #Sets the pose of the next waypoint goal.
    def setNextWaypoint(self):
        #Checks that the waypoint ID is valid.
        if self.curWaypoint >= len(self.waypoints):
            self.curWaypoint = 0
        
        #Gets the waypoint from the array.
        waypoint = self.waypoints[self.curWaypoint]
        
        #Starts creating the goal.
        goal = MoveBaseGoal()
        #Sets the frame ID of the goal, notably the map topic.
        goal.target_pose.header.frame_id = 'map'
        #Sets the postion and orientation of the goal pose.
        goal.target_pose.pose.position.x = waypoint[0][0]
        goal.target_pose.pose.position.y = waypoint[0][1]
        goal.target_pose.pose.position.z = waypoint[0][2]
        goal.target_pose.pose.orientation.x = waypoint[1][0]
        goal.target_pose.pose.orientation.y = waypoint[1][1]
        goal.target_pose.pose.orientation.z = waypoint[1][2]
        goal.target_pose.pose.orientation.w = waypoint[1][3]
        
        #Logs that the next waypoint has been set.
        self.log.publish("Moving to Waypoint: " + waypoint[2])      
        print("Moving to Waypoint: " + waypoint[2])
        
        #Sends the goal to the action client and provides a callback for when
        #the goal is reached, cancelled, or aborted.
        self.client.send_goal(goal, done_cb = self.waypointReached)    
        
    #Timeout function called when beginning to move towards a target.
    def targetTimeout(self):
        #Waits 10 seconds before stopping to move towards the targets.
        #Prevents any errors when the robot cannot reach the target properly.
        rospy.sleep(10)
        
        #Stops the robot from looking for the coloured targets.
        self.targetSearch = False
        
        #Sets the next waypoint.
        self.setNextWaypoint()
        
        #Waits before allowing the robot to look for the targets again.
        rospy.sleep(3)
        self.targetSearch = True
        
    #Called when a waypoint goal is finished.
    def waypointReached(self, state, result):
        #If the waypoint has been reached successfully set the next waypoint.
        if state == actionlib.GoalStatus.SUCCEEDED:
            self.log.publish("Waypoint Reached")   
            print("Waypoint Reached")   
                   
            self.curWaypoint += 1
            self.setNextWaypoint()
            
        #If the waypoint goal has been cancelled a target has been found.
        #Calls the timeout function in case of errors.
        elif state == actionlib.GoalStatus.PREEMPTED:
            self.log.publish("Waypoint Cancelled")
            print("Waypoint Cancelled")
            
            self.targetTimeout()
            
        #If the waypoint has been aborted due to any reason set the goal as the
        #the next waypoint.
        elif state == actionlib.GoalStatus.ABORTED:
            self.log.publish("Waypoint Aborted")
            print("Waypoint Aborted")
            
            self.curWaypoint += 1
            self.setNextWaypoint()
    
    #Called when recieving data from the camera topic.
    def callbackImage(self, data):
        #Checks whether all of the objects have been found and ends the program if all found.
        if self.targets[0][1] and self.targets[1][1] and self.targets[2][1] and self.targets[3][1]:
            #Deregisters the subscribers and publishers.
            self.laser.unregister()
            self.cam.unregister()
            
            #Cancels all waypoint goals.
            self.client.cancel_all_goals()
            
            #Deregisters the subscribers and publishers
            self.log.publish("All objects found.")
            print("All objects found.")
            
            self.log.unregister()    
            
            rospy.signal_shutdown("Program completed.")
            
        else:
            #Enclosed in an try statement to catch any errors caused relating to OpenCV.
            try:
                #Uses the camera data to create an image for processing.
                img = self.bridge.imgmsg_to_cv2(data, "bgr8")
                #Converts the image to HSV to allow for colour slicing.            
                imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
                
                #Gets the dimensions of the image.
                h, w, d = img.shape
            
                #Creates the masks of each object colour using the thresholds.
                masks = [cv2.inRange(imgHSV, self.targets[0][2], self.targets[0][3]),
                         cv2.inRange(imgHSV, self.targets[1][2], self.targets[1][3]),
                         cv2.inRange(imgHSV, self.targets[2][2], self.targets[2][3]), 
                         cv2.inRange(imgHSV, self.targets[3][2], self.targets[3][3])]
                #Combines the masks into one for quicker processing when looking for a coloured object.
                mask = masks[0] + masks[1] + masks[2] + masks[3]
                
                #Limits the robot's view to objects only close preventing it from finding
                #objects far away and potentially blocked by other non target objects not detected.
                search_top = 3*h/4 - 40
                search_bot = 3*h/4 + 40
                mask[0:search_top, 0:w] = 0 
                mask[search_bot:h, 0:w] = 0
                
                #Checks the mask for any moments of interest.
                momentsAll = cv2.moments(mask)
                    
                #If any moments of interest found and if currently looking for the objects.
                if momentsAll['m00'] > 0 and self.targetSearch:
                    #Loops through each target object colour.
                    for colID in range(len(masks)):
                        if not self.targets[colID][1]:
                            #Creates a new mask of the current target colour.
                            moments = cv2.moments(masks[colID])
                            
                            #Determines if any moments have been found colour specific mask.
                            if moments['m00'] > 0:
                                #If the distance is greater than a metre away move towards the object.
                                if self.distance > 1:  
                                    self.log.publish("Moving towards target: " + self.targets[colID][0])
                                    print("Moving towards target: " + self.targets[colID][0])
                                    
                                    #Cancels the current waypoint goal.
                                    self.client.cancel_all_goals()
                        
                                    #Used to calculate the angle the robot needs to move to get to the object.
                                    cx = int(moments['m10']/moments['m00'])                               
                                    err = cx - w/2
                                    
                                    #Creates the movement command for the robot to reach the target object.
                                    self.twist.linear.x = 0.5
                                    self.twist.angular.z = -float(err) / 100
                                
                                    #Publishes the movement goal.
                                    self.veloc.publish(self.twist)
                                    
                                    break
                                #If the distance is less than a metre away the object has been found.
                                elif self.distance <= 1: 
                                    self.log.publish("Found target object: " + self.targets[colID][0])
                                    print("Found target object: " + self.targets[colID][0])                                    
                                    
                                    #Set that the coloured object has been found.
                                    self.targets[colID][1] = True
                                    
                                    rospy.sleep(2)
                                    
                                    #Start moving towards the next waypoint.
                                    self.curWaypoint += 1
                                    self.setNextWaypoint()
                                    
                                    break
                        
            #Prints any errors caused.
            except CvBridgeError, e:
                print e
            
    #Determines the distance away from the target object using the laser scanner.
    def callbackScan(self, data):   
        self.distance = data.ranges[len(data.ranges)/2]

if __name__ == '__main__':    
    rospy.init_node('assignment')
    
    assignment()
    
    rospy.spin()
