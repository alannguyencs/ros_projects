#!/usr/bin/env python3
import rospy
from turtlesim.msg import Pose

def pose_callback(pose_message):
    rospy.loginfo("Robot X = %f: ", pose_message.x)
    rospy.loginfo("Robot Y = %f: ", pose_message.y)
    rospy.loginfo("Robot Theta = %f: ", pose_message.theta)

if __name__=='__main__':
    rospy.init_node('turtle_pose_subscriber')
    sub = rospy.Subscriber('/turtle1/pose', Pose, callback=pose_callback)

    rospy.loginfo("Node has been started")
    rospy.spin() # spin() simply keeps python from exiting until this node is stopped