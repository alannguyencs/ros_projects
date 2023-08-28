#! /usr/bin/env python3

import imageio.v3 as iio
import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
import time

output_path = "/media/alan/新增磁碟區/toshiba/ros_projects/output/"
# Define a global variable to store the depth image data
depth_image_data = []

def depth_image_callback(msg):
    print ('msg', dir(msg))
    print ('encoding', msg.encoding) #the standard one is: 32FC1
    # Convert the ROS Image message to a NumPy array
    bridge = CvBridge()
    try:
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    except Exception as e:
        rospy.logerr(e)
        return
    
    # print properties:
    print(f"Image resolution: {depth_image.shape}")
    print(f"Data type: {depth_image.dtype}")
    print(f"Min value: {np.min(depth_image)}")
    print(f"Max value: {np.max(depth_image)}")

    # Create a mask to identify NaN values
    nan_mask = np.isnan(depth_image)

    # Use numpy.where to get the indices of non-NaN values
    non_nan_indices = np.where(~nan_mask)

    # Access the non-NaN depth values using the indices
    depth_image_filter = np.zeros(depth_image.shape)
    depth_image_filter[non_nan_indices] = depth_image[non_nan_indices]
    print ("min", np.min(depth_image_filter), "max", np.max(depth_image_filter))
    SCALE = 5000 #0x0fff
    print ("scale:", SCALE)

    depth_instensity = np.array(256 * depth_image_filter * 1000 / SCALE,
                            dtype=np.uint8)
    
    print ("min", np.min(depth_instensity), "max", np.max(depth_instensity))
    time_ = time.time()
    print (f"save image to {output_path}/grayscale_{time_}.png")
    iio.imwrite(f'{output_path}/grayscale_{time_}.png', depth_instensity)


    #calibration matrix: rostopic echo /camera/depth/camera_info
    FX_DEPTH = 1206.8897719532354
    FY_DEPTH = 1206.8897719532354
    CX_DEPTH = 960.5
    CY_DEPTH = 540.5
    depth_image = depth_image_filter.copy()
    # compute point cloud:
    pcd = []
    height, width = depth_image.shape
    for i in range(height):
        for j in range(width):
            z = depth_image[i][j]
            x = (j - CX_DEPTH) * z / FX_DEPTH
            y = (i - CY_DEPTH) * z / FY_DEPTH
            pcd.append([x, y, z])
            
    pcd_o3d = o3d.geometry.PointCloud()  # create point cloud object
    pcd_o3d.points = o3d.utility.Vector3dVector(pcd)  # set pcd_np as the point cloud points
    # Visualize:
    o3d.visualization.draw_geometries([pcd_o3d])

def main():
    # Initialize the ROS node
    rospy.init_node('depth_image_subscriber', anonymous=True)

    # Subscribe to the depth image topic
    depth_image_topic = "/camera/depth/image_raw"  # Replace with the actual topic name
    rospy.Subscriber(depth_image_topic, Image, depth_image_callback)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    main()