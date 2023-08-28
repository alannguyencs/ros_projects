#!/usr/bin/env python3

import math, rospy
from utilities import spawn_coke_can, spawn_jersey_barrier, spawn_cylinder, spawn_person_standing, \
                      pause_physics, unpause_physics, delete_model, set_model_state
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_about_axis
import os
import xml.etree.ElementTree as ET

JERSEY_BARRIER_LENGTH = 1.74
CYLINDER_RADIUS = 0.56
NUM_UNITS_SHORT = 2
NUM_UNITS_LONG = 10
UNIT_LENGTH = JERSEY_BARRIER_LENGTH * 2 + CYLINDER_RADIUS
WORLD_DIR = '/home/alan/GoogleDrive/CAiRS/ros_projects/catkin_ws/src/studying_gazebo/worlds/'
WORLD_NAME = 'world_I'

def rotate_object(object_name, position, angle):
    theta = math.radians(angle)
    q = quaternion_about_axis(theta, (0,0,1))
    orientation = Quaternion(*q)
    set_model_state(object_name, Pose(position, orientation))

def build_a_wall(start_x, start_y, num_inits, axis='x', object_id=0):
    #num_units is the number of pairs of jersey barriers and a cylinder
    assert(axis in ['x', 'y'])
    for i in range(num_inits):
        if axis == 'x':
            start_x_i = start_x + i * UNIT_LENGTH + JERSEY_BARRIER_LENGTH / 2
            #first jersey_barrier
            name = f'object_{object_id}'
            position = Point(start_x_i, start_y, 0)
            orientation = Quaternion(0,0,0.7071,0.7071) #sin 45, cos 45
            spawn_jersey_barrier(name, Pose(position=position, orientation=orientation), color='orange')
            rospy.sleep(0.2)
            # rotate_object(name, position, 90)

            #second jersey_barrier
            name = f'object_{object_id+1}'
            position = Point(start_x_i + JERSEY_BARRIER_LENGTH, start_y, 0)
            spawn_jersey_barrier(name, Pose(position=position, orientation=orientation), color='white')
            rospy.sleep(0.2)
            # rotate_object(name, position, 90)

            #cylinder
            spawn_cylinder(f'object_{object_id+2}',
                   Pose(position=Point(start_x_i + 2 * JERSEY_BARRIER_LENGTH - CYLINDER_RADIUS, start_y, 0)))
            
        elif axis == 'y':
            start_y_i = start_y + i * UNIT_LENGTH + JERSEY_BARRIER_LENGTH / 2
            #first jersey_barrier
            spawn_jersey_barrier(f'object_{object_id}', 
                         Pose(position=Point(start_x, start_y_i, 0)),
                         color='orange')
            #second jersey_barrier
            spawn_jersey_barrier(f'object_{object_id+1}', 
                         Pose(position=Point(start_x, start_y_i + JERSEY_BARRIER_LENGTH, 0)),
                         color='white')
            #cylinder
            spawn_cylinder(f'object_{object_id+2}',
                   Pose(position=Point(start_x, start_y_i + 2 * JERSEY_BARRIER_LENGTH - CYLINDER_RADIUS, 0)))
        
        object_id += 3
    return object_id

def build_wall_parallel(start_x, start_y, axis='x', object_id=0):
    object_id = build_a_wall(start_x, start_y, NUM_UNITS_LONG, axis=axis, object_id=object_id)
    object_id = build_a_wall(start_x + NUM_UNITS_SHORT * UNIT_LENGTH, start_y, NUM_UNITS_LONG, axis=axis, object_id=object_id)

    opposite_axis = 'y' if axis == 'x' else 'x'
    object_id = build_a_wall(start_x, start_y, NUM_UNITS_SHORT, axis=opposite_axis, object_id=object_id)
    object_id = build_a_wall(start_x, start_y + NUM_UNITS_LONG * UNIT_LENGTH, NUM_UNITS_SHORT, axis=opposite_axis, object_id=object_id)
    return object_id

def build_world_I(num_units_short=2, num_units_long=10, empty_world=True):
    object_id = build_wall_parallel(start_x = 0,
                                    start_y= 0,
                                    axis = 'y',
                                    object_id = 0)
    if empty_world: return

    spawn_person_standing(f'object_{object_id}', 
                          Pose(position=Point(UNIT_LENGTH * 0.5,
                                                UNIT_LENGTH * NUM_UNITS_LONG * 0.25,
                                                0)))
    spawn_person_standing(f'object_{object_id + 1}', 
                          Pose(position=Point(UNIT_LENGTH * 1.5,
                                                UNIT_LENGTH * NUM_UNITS_LONG * 0.75,
                                                0)))

def generate_waypoint_element(time_, x, y, angle):
    waypoint_element = ET.Element('waypoint')
    time_element = ET.Element('time')
    time_element.text = f"{time_}"
    pose_element = ET.Element('pose')
    pose_element.text = f"{x} {y} 0 0 0 {angle}"
    waypoint_element.append(time_element)
    waypoint_element.append(pose_element)
    return waypoint_element

def generate_person_walking_script(walking_person_id=0):
    template_path = f"{WORLD_DIR}actors/walking_person_template.xml"
    tree = ET.parse(template_path)
    root = tree.getroot()
    root.set('name', 'walking_person_{walking_person_id}')
    script_element = root.find('script')
    trajectory_element = script_element.find('trajectory')

    x = NUM_UNITS_SHORT * UNIT_LENGTH / 2
    y_start = NUM_UNITS_LONG * UNIT_LENGTH * 0.25
    y_finish = NUM_UNITS_LONG * UNIT_LENGTH * 0.95

    time_ = 0
    angle = math.pi * 0.5
    waypoint_element = generate_waypoint_element(time_, x, y_start, angle)
    trajectory_element.append(waypoint_element)
    
    time_ += 10
    angle = math.pi * 0.5
    x = NUM_UNITS_SHORT * UNIT_LENGTH / 2
    y = NUM_UNITS_LONG * UNIT_LENGTH * 0.95
    waypoint_element = generate_waypoint_element(time_, x, y_finish, angle)
    trajectory_element.append(waypoint_element)

    time_ += 2
    angle = math.pi * 1.5
    waypoint_element = generate_waypoint_element(time_, x, y_finish, angle)
    trajectory_element.append(waypoint_element)

    time_ += 10
    angle = math.pi * 1.5
    waypoint_element = generate_waypoint_element(time_, x, y_start, angle)
    trajectory_element.append(waypoint_element)

    time_ += 2
    angle = math.pi * 0.5
    waypoint_element = generate_waypoint_element(time_, x, y_start, angle)
    trajectory_element.append(waypoint_element)

    tree.write(f"{WORLD_DIR}actors/walking_person_{walking_person_id}.xml")

def add_person_walking(walking_person_id=0, pre_world_id=0, post_world_id=1):
    world_path = f"{WORLD_DIR}{WORLD_NAME}_{pre_world_id}.world"
    actor_path = f"{WORLD_DIR}actors/walking_person_{walking_person_id}.xml"

    tree_actor = ET.parse(actor_path)
    root_actor = tree_actor.getroot()
    # world_element = root_actor.find('world')
    # element_actor = world_element.find('actor')
    # print (element_actor)

    tree = ET.parse(world_path)
    root = tree.getroot()

    world_element = root.find('world')
    world_element.insert(1, root_actor)

    tree.write(f"{WORLD_DIR}{WORLD_NAME}_{post_world_id}.world")


if __name__=='__main__':
    unpause_physics()
    # for object_id in range(100):
    #     delete_model(f'object_{object_id}')

    print ('unit_length', UNIT_LENGTH)

    #step 1: build the world
    #roslaunch gazebo_ros empty_world.launch
    # build_world_I()
    # build_world_I(empty_world=False)

    #step 2: generate the script for the person walking
    generate_person_walking_script(walking_person_id=0)
    add_person_walking(walking_person_id=0, pre_world_id=2, post_world_id=3)
    
    #step 3: fix the post_world_id in spawn_world_I.launch
    #to test: 
    # stop the simulation
    # roslaunch studying_gazebo spawn_world_I.launch



    