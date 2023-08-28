#! /usr/bin/env python3

"""
The robot runs ahead and turn to where the distance is the longest.
"""

import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

INF_DISTANCE = 100000.
FRONT_OBSTACLE_DISTANCE = 0.8
FORWARD_SPEED = 0.5
ANGULAR_SPEED = 0.2
SQUARE_ANGLE = 90.
LIDAR_WINDOW = 180
# logging_file = open('230714_move_turn_left.log', 'w')

def callback(msg):
    num_points = len(msg.ranges)
    lidar_data = []
    for i in range(num_points):
        if not math.isinf(msg.ranges[i]):
            lidar_data.append(msg.ranges[i])
        else:
            lidar_data.append(INF_DISTANCE)
    #     logging_file.write('{:.2f}, '.format(lidar_data[i]))
    # logging_file.write('\n')
    # logging_file.flush()

    distance_list = []
    for i in range(num_points):
        left = max(0, i - LIDAR_WINDOW // 2)
        right = min(num_points, i + LIDAR_WINDOW // 2)
        distance_list.append(sum(lidar_data[left:right]) / (right - left)) 
                                        #    + num_points - abs(i - num_points // 2))
    
    max_index = distance_list.index(max(distance_list))
    max_angular = (max_index - len(distance_list) // 2) / (len(distance_list) / 2) * SQUARE_ANGLE
    print ('max_angular = {:.0f}, distance = {}'.format(max_angular, distance_list[max_index]))
    turn_direction = 'LEFT' if max_index < num_points // 2 else 'RIGHT'

    move.linear.x = FORWARD_SPEED
    move.angular.z = ANGULAR_SPEED if turn_direction == 'RIGHT' else -ANGULAR_SPEED
    pub.publish(move)

#Create a Publisher that writes into the /cmd_vel topic in order to move the robot.
rospy.init_node('navigation_230714_node')
# sub = rospy.Subscriber('/scan', LaserScan, callback)
sub = rospy.Subscriber('/front/scan', LaserScan, callback)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
rate = rospy.Rate(2)
move = Twist()
rospy.spin()