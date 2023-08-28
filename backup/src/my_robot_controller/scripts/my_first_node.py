#!/usr/bin/env python3
import rospy

if __name__=='__main__':
    rospy.init_node('test_node')
    rospy.loginfo("Node has started")
    # rospy.logwarn("This is a warning")
    # rospy.logerr("This is an error")

    # rospy.sleep(0.1)
    # rospy.loginfo("Node has ended")

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rospy.loginfo("Hello")
        rate.sleep()
