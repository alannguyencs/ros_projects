#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import math
import os
from geometry_msgs.msg import Twist

INF_DISTANCE = 20.
FRONT_OBSTACLE_DISTANCE = 0.8
FORWARD_SPEED = 0.5
ANGULAR_SPEED = 0.1
SQUARE_ANGLE = 90.
LIDAR_WINDOW = 15

def within_the_box(pt, box=[(-3, 3), [-2, 0.27], [0, 5]]):
    for i in range(3):
        if pt[i] < box[i][0] or pt[i] > box[i][1]: 
            return False
    return True

def compute_depth_data(point_list):
    depth_data = []
    for i in range(180):
        if i < 60 or i > 120: 
            depth_data.append(0)
        else:
            depth_data.append(INF_DISTANCE)

    for point in point_list:
        angle = int(math.atan2(point[2], point[0]) / math.pi * 180)
        assert(angle >= 0 and angle <= 180)
        depth_value = math.sqrt(point[0] ** 2 + point[2] ** 2)
        if depth_value < depth_data[angle]:
            depth_data[angle] = round(depth_value, 2)
    return depth_data

def callback(msg):
    # Process the point cloud data
    # You can access the data and its properties here
    # print (msg.header)
    # print ('height', msg.height)
    # print ('width', msg.width)
    fields = msg.fields
    # print (type(fields), len(fields))
    # print (type(msg.data), len(msg.data))
    point_cloud_generator = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    # print ('point_cloud', type(point_cloud_generator))
    point_list = list(point_cloud_generator)
    # print ('point_list before:', len(point_list))
    # point_list = [point for point in point_list if within_the_box(point)]
    # print ('point_list after:', len(point_list))

    coor = 'xyz'
    for coor_id in range(3):
        point_coor = [round(point[coor_id], 2) for point in point_list]
        point_coor.sort()
        print ('point_{}:'.format(coor[coor_id]), point_coor[:5], point_coor[-5:])
        log_path = f'{coor[coor_id]}_230721.log'
        if not os.path.exists(log_path):
            log_file = open(f'{coor[coor_id]}_230721.log', 'w')
            for point in point_coor:
                log_file.write('{:.2f}\n'.format(point))
            log_file.close()
    
    
    depth_data = compute_depth_data(point_list)
    # count_depth_data = sum([int(i > 0) for i in depth_data if i > 0])
    # print ('count_depth_data', count_depth_data)
    # if count_depth_data < 30:
    #     return
    log_path = 'depth_230721.log'
    if not os.path.exists(log_path):
        log_file = open(log_path, 'w')
        for depth in depth_data:
            log_file.write('{:.2f}\n'.format(depth))
        log_file.close()
    # for i in range(180):
    #     print (i, depth_data[i])
    

    distance_list = []
    num_points = 180
    for i in range(num_points):
        left = max(0, i - LIDAR_WINDOW // 2)
        right = min(num_points, i + LIDAR_WINDOW // 2)
        distance_list.append(sum(depth_data[left:right]) / (right - left))

    log_path = 'distance_230721.log'
    if not os.path.exists(log_path):
        log_file = open(log_path, 'w')
        for distance in distance_list:
            log_file.write('{:.2f}\n'.format(distance))
        log_file.close()
    # for i in range(num_points):
    #     print ('distance', i, distance_list[i])
    
    max_index = distance_list.index(max(distance_list))
    print ('max_index', max_index)
    # max_angular = (max_index - len(distance_list) // 2) / (len(distance_list) / 2) * SQUARE_ANGLE
    max_angular = max_index - 90
    print ('max_angular = {:.0f}, distance = {}'.format(max_angular, distance_list[max_index]))
    turn_direction = 'RIGHT' if max_index < num_points // 2 else 'LEFT'

    # hyper_param = 50
    # # move.linear.x = 0
    # move.angular.z = max_angular / 90 * ANGULAR_SPEED * hyper_param
    print ('move.angular.z = {:.2f}'.format(move.angular.z))
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


rospy.init_node('navigation_230721_node', anonymous=True)
sub = rospy.Subscriber('/points2', PointCloud2, callback)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
rate = rospy.Rate(2)
move = Twist()
rospy.spin()
