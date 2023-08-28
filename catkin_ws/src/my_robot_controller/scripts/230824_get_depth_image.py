#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

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

    # Create a mask to identify NaN values
    nan_mask = np.isnan(depth_image)

    # Use numpy.where to get the indices of non-NaN values
    non_nan_indices = np.where(~nan_mask)

    # Access the non-NaN depth values using the indices
    non_nan_depth_values = depth_image[non_nan_indices]
    print ("min", np.min(non_nan_depth_values), "max", np.max(non_nan_depth_values))

    mean_depth = np.mean(non_nan_depth_values)
    print("Mean Depth Value:", mean_depth)

    # # Now you can work with the NumPy array (cv_image)
    # # For example, you can access individual pixel values
    # print ('cv_image.shape:', cv_image.shape)
    # # print ('min', np.min(cv_image), 'max', np.max(cv_image))
    # print (cv_image[10][10])

    # Or perform image processing operations using OpenCV
    # For example, convert the image to grayscale
    # grayscale_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

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