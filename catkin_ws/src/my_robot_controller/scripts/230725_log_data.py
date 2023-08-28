#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

def point_cloud_callback(msg):
    # Process the point cloud data
    # You can access the data and its properties here
    print (msg.header)
    print ('height', msg.height)
    print ('width', msg.width)
    fields = msg.fields
    print (type(fields), len(fields))
    print (type(msg.data), len(msg.data))
    point_cloud_generator = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    print ('point_cloud', type(point_cloud_generator))
    point_list = list(point_cloud_generator)
    print ('point_list', len(point_list), type(point_list[0]))

    coor = 'xyz'
    for coor_id in range(3):
        point_coor = [round(point[coor_id], 2) for point in point_list]
        point_coor.sort()
        print ('point_{}:'.format(coor[coor_id]), point_coor[:5], point_coor[-5:])
    # point_z = [round(point[2], 2) for point in point_list]
    # point_z.sort()
    # print ('point_z', point_z[:10], point_z[-10:])
    # for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
    #     x, y, z = point
    #     # Process the individual x, y, and z coordinates of each point
    #     # Your custom processing code goes here
    #     print("Point: x={}, y={}, z={}".format(x, y, z))

def listener():
    rospy.init_node('point_cloud_listener', anonymous=True)
    rospy.Subscriber('/points2', PointCloud2, point_cloud_callback)
    rate = rospy.Rate(1) # 2hz
    rospy.spin()

if __name__ == '__main__':
    listener()
