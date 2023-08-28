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

def to_radian(degrees):
    return degrees / 180.0 * 3.14

def to_degree(radian):
    return radian / 3.14 * 180.0

def rotate(theta_target, max_vel=0.5, eps=0.01):
    while True:
        theta = odometry.odom_pose['theta']
        theta_diff = theta_target - theta
        # print ('theta_diff = {:.2f}'.format(theta_diff))
        if abs(theta_diff) < 0.01: break
            
        if theta_diff > 0:
            vel_rotation = min(max_vel, theta_diff)
        else:
            vel_rotation = max(-max_vel, theta_diff)

        velocity.move(0, vel_rotation)
        rospy.sleep(0.01)

def get_heading_theta(x_target, y_target):
    dx = x_target - odometry.odom_pose['x']
    dy = y_target - odometry.odom_pose['y']
    theta_target_ = np.arctan2(dy, dx)
    return theta_target_

def go_straight(x_target, y_target, theta_target, max_vel=0.5, max_vel_rotation=0.25, eps=0.01):
    cnt = 0
    while True:
        #step 1: make sure the robot is facing the target within a certain error
        # if cnt % 10 == 0:
        #     theta_target = get_heading_theta(x_target, y_target)
        #     rotate(theta_target, eps=0.5)
        
        theta_target = get_heading_theta(x_target, y_target)
        theta_diff = theta_target - odometry.odom_pose['theta']
        if theta_diff > 0:
            vel_rotation = min(max_vel, theta_diff)
        else:
            vel_rotation = max(-max_vel, theta_diff)

        #step 2: move forward
        dx = x_target - odometry.odom_pose['x']
        dy = y_target - odometry.odom_pose['y']
        dist = np.sqrt(dx**2 + dy**2)
        if dist < eps: break

        vel_forward = min(max_vel, dist)
        velocity.move(vel_forward, vel_rotation)
        rospy.sleep(0.01)

        cnt += 1
        # if cnt % 50 == 0:
        #     print ("Current position (%.2f, %.2f, %.0f)" % (odometry.odom_pose['x'], odometry.odom_pose['y'], to_degree(odometry.odom_pose['theta'])))
        #     print ("Reaching (%.2f, %.2f, %.0f)\n" % (x_target, y_target, to_degree(theta_target)))
        #     print ("Distance: {:.2f}".format(dist))

def go_to(x_target, y_target, theta_target):
    theta_target_ = get_heading_theta(x_target, y_target)

    #step 1: facing to point (x_target, y_target)
    print ('Rotating to {:.0f}'.format(to_degree(theta_target_)))
    rotate(theta_target_)
    print ("Done step 1:", "Current position (%.2f, %.2f, %.0f)\n" % (odometry.odom_pose['x'], odometry.odom_pose['y'], to_degree(odometry.odom_pose['theta'])))

    #step 2: move forward
    print ("Heading to (%.2f, %.2f, %.0f)" % (x_target, y_target, to_degree(theta_target_)))
    go_straight(x_target, y_target, theta_target_)
    print ("Done step 2:", "Current position (%.2f, %.2f, %.0f)\n" % (odometry.odom_pose['x'], odometry.odom_pose['y'], to_degree(odometry.odom_pose['theta'])))

    #step 3: facing to theta_target
    theta_target = to_radian(theta_target)
    print ('Rotating to {:.0f}'.format(to_degree(theta_target)))
    rotate(theta_target)
    print ("Bingo! Reached (%.2f, %.2f, %.0f)" % (x_target, y_target, to_degree(theta_target)))


# def go_to(xg, yg, thetag_degrees, constant_vel = None, max_vel=None):
#     rho = float("inf")
#     thetag = math.radians(thetag_degrees)
#     cnt = 0
#     while rho>0.01:
#         dx = xg - odometry.odom_pose['x']
#         dy = yg - odometry.odom_pose['y']
#         rho = np.sqrt(dx**2 + dy**2)
#         theta = odometry.odom_pose['theta']
#         alpha = normalize(np.arctan2(dy, dx) - theta)
#         beta = normalize(thetag - np.arctan2(dy, dx))
#         v = k_rho * rho
#         w = k_alpha * alpha + k_beta * beta
#         if constant_vel:
#             abs_v = abs(v)
#             v = v / abs_v * constant_vel
#             w = w / abs_v * constant_vel
#         if abs(v) > max_vel:
#             abs_v = abs(v)
#             v = v / abs_v * max_vel
#             w = w / abs_v * max_vel
#         velocity.move(v, w)
#         rospy.sleep(0.01)

#         cnt += 1
#         if cnt % 50 == 0:
#             print ("Current position (%.2f, %.2f, %.2f)" % (odometry.odom_pose['x'], odometry.odom_pose['y'], odometry.odom_pose['theta'] / 3.14 * 180))
#             print ("Reaching (%.2f, %.2f, %.2f)\n" % (xg, yg, thetag_degrees))
#     print ("Bingo! Reached (%.2f, %.2f, %.2f)" % (xg, yg, thetag_degrees))

k_rho = 0.3
k_alpha = 0.8
k_beta = -0.15

velocity = VelocityController('/cmd_vel')
odometry = OdometryReader('/odom')

# waypoints = [(1,-1,-90),(2,-2,0),(3,-2,0),(4,-1,90),(3.5,-0.5,180),
#              (3,0,90),(3,1,90),(2,1,-90),(1,0,180),(0,0,180)]

# waypoints = [(4, 1, 180), (3, 10, 180), (5, 30, 180)]
# waypoints = [(4, 1, 180), (2, 6, 90)]
waypoints = [(4, 1, 180), (2, 6, 90)]

# waypoints = []
# dx, dy = 2, 2
# for i in range(10):
#     if i % 2 == 0:
#         dx += 1
#         theta = 90
#     else:
#         dy += 1
#         theta = 0
    
#     waypoints.append((dx, dy, theta))


print ("Starting point (%.2f, %.2f, %.2f)" % (odometry.odom_pose['x'], odometry.odom_pose['y'], to_degree(odometry.odom_pose['theta'])))

# rotate(3.14)

for x, y, theta in waypoints:
    print ("Going to (%.2f, %.2f, %.2f)" % (x, y, theta))
    # go_to(xg, yg, thetag, constant_vel=0.3)
    go_to(x, y, theta)
    
velocity.move(0,0)
odometry.unregister()
# error = math.hypot(odometry.odom_pose['x'], odometry.odom_pose['y'])
# print('Final positioning error is %.2fm' % error)