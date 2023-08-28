#!/usr/bin/env python3

import math, rospy
from utilities import spawn_coke_can, spawn_jersey_barrier, spawn_cylinder, \
                      pause_physics, unpause_physics
from geometry_msgs.msg import Pose, Point, Quaternion

JERSEY_BARRIER_LENGTH = 1.74
CYLINDER_RADIUS = 0.56



if __name__=='__main__':
    unpause_physics()
    # spawn_jersey_barrier('jersey_barrier_4', 
    #                      Pose(position=Point(0,-1,2)),
    #                      color='orange')
    # spawn_jersey_barrier('jersey_barrier_1', 
    #                      Pose(position=Point(0,JERSEY_BARRIER_LENGTH,0)),
    #                      color='white')
    # spawn_coke_can('coke_can_1', 
    #                      Pose(position=Point(0,1,1)))
    
    spawn_cylinder('cylinder_1',
                   Pose(position=Point(0,0,0)))
    spawn_cylinder('cylinder_2',
                   Pose(position=Point(CYLINDER_RADIUS,0,0)))