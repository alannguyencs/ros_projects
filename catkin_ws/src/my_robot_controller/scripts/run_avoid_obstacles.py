#! /usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

DISTANCE_TO_OBSTACLE_THRESHOLD = 1.
FORWARD_SPEED = 0.5
ANGULAR_SPEED = 0.1

#Create a Subscriber that reads from the /kobuki/laser/scan topic.
def callback(msg):
    move.linear.x = 0.2
    move.angular.z = 0

    num_points = len(msg.ranges)

    #front
    mid_point = num_points // 2
    front_distance = msg.ranges[mid_point]
    if not math.isinf(front_distance) and front_distance < 1.0:
        move.linear.x = 0
        move.angular.z = ANGULAR_SPEED
    
    #right
    right_distance = msg.ranges[0]
    if not math.isinf(right_distance) and right_distance < 1.0:
        move.linear.x = 0
        move.angular.z = ANGULAR_SPEED

    #left
    left_distance = msg.ranges[-1]
    if not math.isinf(left_distance) and left_distance < 1.0:
        move.linear.x = 0
        move.angular.z = -ANGULAR_SPEED
    
    pub.publish(move)

#Create a Publisher that writes into the /cmd_vel topic in order to move the robot.
rospy.init_node('topics_quiz_node')
# sub = rospy.Subscriber('/scan', LaserScan, callback)
sub = rospy.Subscriber('/front/scan', LaserScan, callback)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
rate = rospy.Rate(2)
move = Twist()
rospy.spin()