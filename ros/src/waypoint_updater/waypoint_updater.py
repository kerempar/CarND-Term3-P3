#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math

from scipy.spatial import KDTree
import numpy as np

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
TARGET_TOP_SPEED_MPH = -1
MAX_DECEL = 0.5

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        print("init waypointupdater")
 
        # Subscribe to topics /current_pose and /base_waypoints
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1)
        rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb, queue_size=1)


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.obstacle_waypoints = None
        self.pose = None
        self.stopline_wp_idx = None

        #rospy.spin()
        self.loop()
        
    def loop(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
           # publish lookahead waypoints
           if self.pose and self.base_waypoints:  
              self.publish_waypoints()
           rate.sleep()


    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg
        #print("pose: " + str(msg))
      

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        if self.base_waypoints is None:
          self.base_waypoints = waypoints
          self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
          self.waypoint_tree = KDTree(self.waypoints_2d)
          print("base waypoints received:", len(self.base_waypoints.waypoints))
          #print("base waypoints:" + str(self.base_waypoints.waypoints))
                

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stopline_wp_idx = msg.data
        #print("stopline_wp_idx: " + str(msg.data))     
    

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        self.obstacle_waypoints = msg.waypoints
        #print("obstacle waypoint: " + str(msg.waypoints)) 


    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x


    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity


    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    
    def distance2(self, a, b):
        dist = math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2) 
        return dist

    
    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]
        
        # check if closest is ahead or behind vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx-1]
        
        # equation for hyperplane through closest coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])
        
        val = np.dot(cl_vect-prev_vect, pos_vect-cl_vect)

        if val > 0:
           closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        return closest_idx
    
  
    def decelerate_waypoints(self, waypoints, closest_idx):
        temp = []

        for i, wp in enumerate(waypoints):
           p = Waypoint()
           p.pose = wp.pose
  
           stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0) # two waypoints back from line so front of car stops at line
           dist = self.distance(waypoints, i, stop_idx)
           vel = math.sqrt(2 * MAX_DECEL * dist) 
	   if vel < 1.:
	      vel = 0.

           p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
           temp.append(p)

        return temp       


    def publish_waypoints(self): 
        # set final waypoints
        final_lane = self.generate_lane()      
        # publish final waypoints
        self.final_waypoints_pub.publish(final_lane)


    def generate_lane(self):
        # build a final_waypoints message
        lane = Lane()
         
        # the first waypoint in the list published to /final_waypoints should be the first waypoint
        # that is currently ahead of the vehicle
        # get closest waypoint
        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        final_waypoints = self.base_waypoints.waypoints[closest_idx:farthest_idx]
       
        # set the velocity for final waypoints in m/sec (for testing)
        if TARGET_TOP_SPEED_MPH > 0:
           for i in range(len(final_waypoints) - 1): 
             self.set_waypoint_velocity(final_waypoints, i, (TARGET_TOP_SPEED_MPH * 1609.34) / (60*60))  
        
        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_idx) or self.stopline_wp_idx == None:
           lane.waypoints = final_waypoints
        else:
           #print("decelerate_waypoints => closest_idx:" + str(closest_idx) + " farthest_idx: " + str(farthest_idx) + " stopline_idx: " + str(self.stopline_wp_idx))
           lane.waypoints = self.decelerate_waypoints(final_waypoints, closest_idx) 
      
        return lane

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
