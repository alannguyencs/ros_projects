#! /usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import LaserScan


def callback(msg):
    num_points = len(msg.ranges)
    print ('Number of points = {}'.format(num_points))
    print ('min = {}'.format(min(msg.ranges)))
    print ('max = {}'.format(max(msg.ranges)))

    #front
    mid_point = num_points // 2
    front_distance = msg.ranges[mid_point]
    right_distance = msg.ranges[0]
    left_distance = msg.ranges[-1]
    print ('front_distance = {}'.format(front_distance))
    print ('right_distance = {}'.format(right_distance))
    print ('left_distance = {}'.format(left_distance))


rospy.init_node('topics_quiz_node')
sub = rospy.Subscriber('/front/scan', LaserScan, callback)
rate = rospy.Rate(2)
rospy.spin()