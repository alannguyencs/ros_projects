#! /usr/bin/env python3

"""
This movement is with an assumption that:
+ Odometry sensors are CORRECT
+ Robot movement is with certain error

Here we create some segment points on the way
and make sure the robot reaches every points
"""

import rospy, math, numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

rospy.init_node('kinematic_controller', anonymous=True)

class VelocityController():
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

def normalize(angle):
    return np.arctan2(np.sin(angle),np.cos(angle))

def go_to(xg, yg, thetag_degrees, constant_vel = None, max_vel=None):
    rho = float("inf")
    thetag = math.radians(thetag_degrees)
    cnt = 0
    while rho>0.01:
        dx = xg - odometry.odom_pose['x']
        dy = yg - odometry.odom_pose['y']
        rho = np.sqrt(dx**2 + dy**2)
        theta = odometry.odom_pose['theta']
        alpha = normalize(np.arctan2(dy, dx) - theta)
        beta = normalize(thetag - np.arctan2(dy, dx))
        v = k_rho * rho
        w = k_alpha * alpha + k_beta * beta
        if constant_vel:
            abs_v = abs(v)
            v = v / abs_v * constant_vel
            w = w / abs_v * constant_vel
        if abs(v) > max_vel:
            abs_v = abs(v)
            v = v / abs_v * max_vel
            w = w / abs_v * max_vel
        velocity.move(v, w)
        rospy.sleep(0.01)

        cnt += 1
        if cnt % 50 == 0:
            print ("Current position (%.2f, %.2f, %.2f)" % (odometry.odom_pose['x'], odometry.odom_pose['y'], odometry.odom_pose['theta'] / 3.14 * 180))
            print ("Reaching (%.2f, %.2f, %.2f)\n" % (xg, yg, thetag_degrees))
    print ("Bingo! Reached (%.2f, %.2f, %.2f)" % (xg, yg, thetag_degrees))

k_rho = 0.3
k_alpha = 0.8
k_beta = -0.15

velocity = VelocityController('/cmd_vel')
odometry = OdometryReader('/odom')

# waypoints = [(1,-1,-90),(2,-2,0),(3,-2,0),(4,-1,90),(3.5,-0.5,180),
#              (3,0,90),(3,1,90),(2,1,-90),(1,0,180),(0,0,180)]

# waypoints = [(4, 1, 180), (3, 10, 180), (5, 30, 180)]
waypoints = [(4, 1, 180)]

print ("Starting point (%.2f, %.2f, %.2f)" % (odometry.odom_pose['x'], odometry.odom_pose['y'], odometry.odom_pose['theta'] / 3.14 * 180))
for xg, yg, thetag in waypoints:
    print ("Going to (%.2f, %.2f, %.2f)" % (xg, yg, thetag))
    # go_to(xg, yg, thetag, constant_vel=0.3)
    go_to(xg, yg, thetag, constant_vel=False, max_vel=0.5)
    
velocity.move(0,0)
odometry.unregister()
error = math.hypot(odometry.odom_pose['x'], odometry.odom_pose['y'])
print('Final positioning error is %.2fm' % error)