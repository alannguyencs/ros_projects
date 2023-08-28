#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import time

if __name__=='__main__':
    rospy.init_node('draw_circle')
    rospy.loginfo("Node has started")

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) # queue_size is the size of the buffer
    rate = rospy.Rate(2) # 2hz
    while not rospy.is_shutdown():
        #publish cmd_vel
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 1.0
        pub.publish(msg)
        print (time.time())
        rate.sleep()