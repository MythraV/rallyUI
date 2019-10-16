#!/usr/bin/env python

#############################################################################
#   waypoint.py - script provides functionality to get clickd points on map
#       in rviz using "2D Nav goal" tool
#   The points are stored in an array and published to a visualisation marker
#   Also provides service to send all set waypoints till now using ros service
#
#   Author                                      Ver
#   --------------------------                  --------------------
#   Voyles? (provided as part of ROS course)    1
#   Mythra                                      1.1 (service and edits to display waypts)
#   Mythra                                      1.2 (Added options to remove waypts in marker)
##############################################################################

import rospy
import numpy as np
import tf
import math
import geometry_msgs.msg
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from msgsrv.srv import waypt

class waypoint(object):
    def __init__(self):
        self.path = Marker()
        self.points = Marker()		
        self.marker_id = 1
        rospy.init_node('waypt_echoer')
		# subscribe to "/move_base_simple/goal" to get picked way points using 2D Nav Goal in rviz
        rospy.Subscriber("/move_base_simple/goal", geometry_msgs.msg.PoseStamped, self.get_way_point)
		# display picked way points and path between way points in rviz
        self.publisher = rospy.Publisher('visualization_marker', Marker, queue_size = 10)

    # waypt service call back
    def send_waypt(self, inp): 
        # Sends waypts on serivice request 
        if inp.req == 1:
            return self.path
        elif inp.req == 0:  #Reset waypoint
            pts_len = len(self.path.points)
            print(pts_len-1)
            self.path.points = self.path.points[0:pts_len-1]
            self.points.points = self.points.points[0:pts_len-1]
            self.publisher.publish(self.path)
            self.publisher.publish(self.points)
            return self.path
        elif inp.req == -1:  #Reset waypoint
            self.path.points = self.path.points[0:0]
            self.points.points = self.points.points[0:0]
            self.publisher.publish(self.path)
            self.publisher.publish(self.points)
            return self.path
        else:
            fuzzypt = self.path     # Replace with empty points
            return fuzzypt
                          
	# fetch clicked way points
    def get_way_point(self, msg):
		# display way points and path on the map
        self.display_way_point(msg.pose.position.x,msg.pose.position.y)
        self.display_path(msg.pose.position.x,msg.pose.position.y)
		# print picked way points in terminal
		# print msg.pose.position.x, msg.pose.position.y

		# get orientationn and convert quternion to euler (roll pitch yaw)
        quaternion = (
	        msg.pose.orientation.x,
	        msg.pose.orientation.y,
	        msg.pose.orientation.z,
	        msg.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = math.degrees(euler[2])
        print "X " , msg.pose.position.x, "m Y ", msg.pose.position.y, " m Yaw ", yaw, "degrees"

# display way points on the map
    def display_way_point(self,x,y):
        self.points.header.frame_id = "/map"	# publish path in map frame		
        self.points.type = self.points.POINTS
        self.points.action = self.points.ADD
        self.points.lifetime = rospy.Duration(0)
        self.points.id = self.marker_id
        self.points.scale.x = 0.2
        self.points.scale.y = 0.2	
        self.points.color.a = 1.0
        self.points.color.r = 0.0
        self.points.color.g = 0.0
        self.points.color.b = 1.0
        self.points.pose.orientation.w = 1.0

        point = Point()
        point.x = x
        point.y = y
        print(self.points.points)
        self.points.points.append(point)
        # Publish the MarkerArray
        self.publisher.publish(self.points)

	# display path between way points on the map
    def display_path(self,x,y):
        self.path.header.frame_id = "/map"	# publish path in map frame
        self.path.type = self.path.LINE_STRIP
        self.path.action = self.path.ADD
        self.path.lifetime = rospy.Duration(0)
        self.path.id = 0
        self.path.scale.x = 0.1
        self.path.color.a = 0.5
        self.path.color.r = 1.0
        self.path.color.g = 0.0
        self.path.color.b = 0.0
        self.path.pose.orientation.w = 1.0

        point = Point()
        point.x = x
        point.y = y

        self.path.points.append(point)
        #print(self.path.points)    #for debug
        # Publish the path
        self.publisher.publish(self.path)

    def run(self):
        # Setup service provider which returns the current points array on request
        self.wayptsrv = rospy.Service('get_waypt', waypt, self.send_waypt)
        rospy.spin()

if __name__ == '__main__':
	print "*********** waypoint.py: read and display way point on the map ***********"
	waypoint().run()
	
