#! /usr/bin/env python

import copy
import actionlib
import rospy

from math import sin, cos, pi
from std_msgs.msg import Header, Int16, String

import armpy
from waypointgathering import gather_waypoints, create_trajectory_from_waypoints

if __name__=="__main__":
    rospy.init_node("waypointgathering")
    #gather_waypoints('test_waypoints.csv')
    create_trajectory_from_waypoints(filename='test_waypoints.csv')



