#! /usr/bin/env python3

"""
The robot runs ahead and turn to where the distance is the longest.
"""

import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
import message_filters

ROSPY_RATE = 2
INF_DISTANCE = 100000.
FRONT_OBSTACLE_DISTANCE = 2.0
FRONT_DEAD_DISTANCE = 0.8
FORWARD_SPEED = 0.5
ANGULAR_SPEED = 0.2
SQUARE_ANGLE = 90.
LIDAR_WINDOW = 180
turn_right_state = False
# logging_file = open('230714_move_turn_left.log', 'w')

def get_rgb_image(msg_rgb):
    # Convert the ROS Image message to a CV2 image
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg_rgb, "bgr8")
    # print ('cv_image', type(cv_image), cv_image.shape)
    red_channel = cv_image[:, :, 2]
    red_percentage = (red_channel > 200).sum() / (red_channel.shape[0] * red_channel.shape[1])
    # print ('red_percentage = {:.2f}'.format(red_percentage))
    return red_percentage

def turn_right():
    move.linear.x = 0
    move.angular.z = -ANGULAR_SPEED
    pub.publish(move)

def callback(msg, msg_rgb):
    # global turn_right_state
    # red_percentage = get_rgb_image(msg_rgb)
    # if red_percentage > 0.5: turn_right_state = True
    # if turn_right_state:
    #     turn_right()
    #     if red_percentage <= 0.05:
    #         turn_right_state = False
    #     else:
    #         return

    num_points = len(msg.ranges)
    mid_point = num_points // 2
    front_depth = sum(msg.ranges[mid_point-10:mid_point+10]) / 20

    # print ('FRONT_DISTANCE', FRONT_DEAD_DISTANCE, sum(msg.ranges[mid_point-10:mid_point+10]) / 20)
    # if sum(msg.ranges[mid_point-10:mid_point+10]) / 20 < FRONT_DEAD_DISTANCE:
    #     move.linear.x = - FORWARD_SPEED / 4
    #     move.angular.z = ANGULAR_SPEED * 4
    #     pub.publish(move)
    #     return

    lidar_data = []
    for i in range(num_points):
        if not math.isinf(msg.ranges[i]):
            lidar_data.append(msg.ranges[i])
        else:
            lidar_data.append(INF_DISTANCE)
    #     logging_file.write('{:.2f}, '.format(lidar_data[i]))
    # logging_file.write('\n')
    # logging_file.flush()

    red_percentage = get_rgb_image(msg_rgb)
    print ('red_percentage = {:.2f}, front_depth = {:.2f}'.format(red_percentage, front_depth))
    if red_percentage > 0.1 and front_depth < FRONT_OBSTACLE_DISTANCE:
        # consider all points on the left hand sise as obstacles
        print ("TURN RIGHT")
        for i in range(num_points // 2, num_points):
            lidar_data[i] = 0
        print (sum(lidar_data[num_points // 2:]), sum(lidar_data[:num_points // 2]))

    distance_list = []
    for i in range(num_points):
        left = max(0, i - LIDAR_WINDOW // 2)
        right = min(num_points, i + LIDAR_WINDOW // 2)
        distance_list.append(sum(lidar_data[left:right]) / (right - left)) 
                                        #    + num_points - abs(i - num_points // 2))
    
    max_index = distance_list.index(max(distance_list))
    max_angular = (max_index - len(distance_list) // 2) / (len(distance_list) / 2) * SQUARE_ANGLE
    print ('max_angular = {:.0f}, distance = {}'.format(max_angular, distance_list[max_index]))
    turn_direction = 'RIGHT' if max_index < num_points // 2 else 'LEFT'

    move.angular.z = ANGULAR_SPEED if turn_direction == 'LEFT' else -ANGULAR_SPEED
    if abs(max_angular) < 10: 
        move.linear.x = FORWARD_SPEED
    elif abs(max_angular) < 22.5: 
        move.linear.x = FORWARD_SPEED / 2
        move.angular.z = move.angular.z * 2
    else: 
        move.linear.x = FORWARD_SPEED / 4
        move.angular.z = move.angular.z * 4
    
    
    pub.publish(move)

#Create a Publisher that writes into the /cmd_vel topic in order to move the robot.
rospy.init_node('navigation_230718_node')
# sub = rospy.Subscriber('/scan', LaserScan, callback)
sub = message_filters.Subscriber('/front/scan', LaserScan) #LaserScan is a message type
sub_rgb = message_filters.Subscriber('/front/left/image_raw', Image) #Image is a message type
ts = message_filters.ApproximateTimeSynchronizer([sub, sub_rgb], queue_size=10, slop=0.5)
ts.registerCallback(callback)

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
rate = rospy.Rate(2) # 2hz
move = Twist()
rospy.spin()