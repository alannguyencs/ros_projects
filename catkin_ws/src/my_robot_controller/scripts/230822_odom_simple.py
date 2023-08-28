#! /usr/bin/env python3

"""
This movement is with an assumption that:
+ Odometry sensors are correct
+ Robot movement is matching with what we command
"""

import rospy, math, numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

rospy.init_node('kinematic_controller', anonymous=True)

class VelocityController():
    #move 10 times with a constant (linear_velocity, angular_velocity)
    def __init__(self, topic):
        self.cmd_vel = rospy.Publisher(topic, Twist, queue_size=10)
        rospy.sleep(0.1)

    def move(self, linear_velocity=0.0, angular_velocity=0.0):
        msg = Twist()
        msg.linear.x = linear_velocity
        msg.angular.z = angular_velocity
        self.cmd_vel.publish(msg)

class OdometryReader():
    def __init__(self, topic):
        self.odom_pose = {}
        self.trajectory = []
        self.topic = topic
        self.subscribe()

    def callback(self, msg):
        self.odom_pose['x'] = msg.pose.pose.position.x
        self.odom_pose['y'] = msg.pose.pose.position.y
        self.trajectory.append((self.odom_pose['x'], self.odom_pose['y']))
        (_, _, self.odom_pose['theta']) = euler_from_quaternion([msg.pose.pose.orientation.x, 
                                                            msg.pose.pose.orientation.y, 
                                                            msg.pose.pose.orientation.z, 
                                                            msg.pose.pose.orientation.w])
    def subscribe(self):
        self.odom_subscriber = rospy.Subscriber(self.topic, Odometry, self.callback)
        rospy.sleep(0.1)

    def unregister(self):
        np.save('trajectory',self.trajectory)
        self.odom_subscriber.unregister()

velocity = VelocityController('/cmd_vel')
odometry = OdometryReader('/odom')
rospy.sleep(1)

##### YOUR CODE STARTS HERE ##### 

# def left(angle=90, radius=1):
#     s = abs(radius)*abs(math.radians(angle))
#     return (abs(radius), s)

# def right(angle=90, radius=1):
#     s = abs(radius)*abs(math.radians(angle))
#     return (-abs(radius), s)

# def straight(s=1):
#     return (float('inf'), s)

def straight(linear_velocity = 0.5, distance = 2.0):
    t = distance / linear_velocity
    velocity.move(linear_velocity = linear_velocity, angular_velocity = 0)
    rospy.sleep(t)
    return odometry.odom_pose['x'], odometry.odom_pose['y']

def turn(angular_velocity = 0.5, angle = 90):
    angle = math.radians(angle)
    t = angle / angular_velocity
    velocity.move(linear_velocity = 0, angular_velocity = angular_velocity)
    rospy.sleep(t)
    return odometry.odom_pose['x'], odometry.odom_pose['y']

print (straight(distance=2.0))
print (turn(angle=90))
print (straight(distance=3.0))

# print ('start_point = ({}, {})'.format(odometry.odom_pose['x'], odometry.odom_pose['y']))
# velocity.move(linear_velocity = -1, angular_velocity = 0)
# rospy.sleep(2)
# print ('finish_point = ({}, {})'.format(odometry.odom_pose['x'], odometry.odom_pose['y']))

# v = 0.65
# path = [right(), left(), straight(), left(), left(radius=0.5), right(radius=0.5),
#         straight(), left(angle=180,radius=0.5), right(), straight()]
# waypoints = []
# waypoints.append((odometry.odom_pose['x'], odometry.odom_pose['y']))

# for (R, s) in path:
#     w = v / R
#     velocity.move(v,w)

#     # we need some time for all moving actioins to be executed
#     t = s / v
#     rospy.sleep(t)
#     waypoints.append((odometry.odom_pose['x'], odometry.odom_pose['y']))

##### YOUR CODE ENDS HERE ##### 
    
velocity.move(0, 0)
odometry.unregister()
# np.save('waypoints',waypoints)
# error = math.hypot(odometry.odom_pose['x'], odometry.odom_pose['y'])
# print('Final positioning error is %.2fm' % error)
"""
set model state:
rosservice call /gazebo/set_model_state '{model_state: { model_name: mobile_base, pose: { position: { x: 0.0, y: 0.0, z: 0.1}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0 } }, reference_frame: world } }'
"""