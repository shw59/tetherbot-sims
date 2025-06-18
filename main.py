"""
main.py

This file contains the main driver program for running the tetherbot PyBullet simulation.
"""

import pybullet as p
import pybullet_data
from world import World
from agent import Agent
import numpy as np
import math
import random

HEIGHT = 0.01
TIME_STEP = 1./240.
N = 5
UNSTRETCHED_TETHER_LENGTH = 1
RADIUS = 0.1
GRADIENT_SOURCE = [0, 0]
# ANGLE_WEIGHT = 2
# STRAIN_WEIGHT = 100
# GRADIENT_WEIGHT = 2
# REPULSION_WEIGHT = 4
ANGLE_WEIGHT = 0
STRAIN_WEIGHT = 100
GRADIENT_WEIGHT = 50
REPULSION_WEIGHT = 4
SENSING_PERIOD = 5 # The number of while loop iterations that should run before a singular, random
                           # agent updates its goal position

def get_starting_positions(l_0, n, angles, left_most_position):
    """
    Calculates the starting position based on the list of desired starting angles and the unstretched length
    of the tether between the agents. "left_most_position" is given as [x, y, z].
    """
    the_list = []

    for i in range(n):
        the_list.append([0])

    the_list[0] = left_most_position

    for i in range(1, n):
        if angles[i-1] == None:
            next_position = [the_list[i-1][0] + 1, the_list[i-1][1], the_list[i-1][2]]
        else:
            prev_theta = np.degrees(math.atan2(the_list[i-2][1] - the_list[i-1][1], the_list[i-2][0] - the_list[i-1][0]))
            new_theta = prev_theta + (360-angles[i-1])
            new_x = the_list[i-1][0] + l_0*np.cos(np.radians(new_theta))
            new_y = the_list[i-1][1] + l_0*np.sin(np.radians(new_theta))
            next_position = [new_x, new_y, 0]

        the_list[i] = next_position

    return the_list

def generate_circular_obstacle_course(world, n_obstacles, obstacle_size_range, course_radius):

    for i in range(n_obstacles):
        r = random.uniform(0, course_radius - (obstacle_size_range[1] / 2))
        theta = random.uniform(0, 2 * math.pi)
        x = r * math.cos(theta)
        y = r * math.sin(theta)
        sizes = random.uniform(obstacle_size_range[0], obstacle_size_range[1])
        no_overlap = True
        for obj in world.obj_list:
            if obj.label == "obstacle" and math.dist(obj.get_pose()[0], [x, y]) <= sizes * 2:
                no_overlap = True
                break
        if no_overlap:
            world.create_obstacle("hexagon", [x, y], length=sizes, width=sizes, color=(0, 1, 0, 1), fixed=True)

def add_axis_labels():
    p.addUserDebugLine([0, 0, 0], [1, 0, 0], lineColorRGB=[0, 0, 1], lineWidth=10, lifeTime=0)
    p.addUserDebugLine([0, 0, 0], [0, 1, 0], lineColorRGB=[1, 0, 0], lineWidth=10, lifeTime=0)
    p.addUserDebugLine([0, 0, 0], [0, 0, 1], lineColorRGB=[.68, .12, .94], lineWidth=10, lifeTime=0)
    p.addUserDebugText("+x", [1, 0, 0], lifeTime=0, textColorRGB=[0, 0, 0])
    p.addUserDebugText("+y", [0, 1, 0], lifeTime=0, textColorRGB=[0, 0, 0])
    p.addUserDebugText("+z", [0, 0, 1], lifeTime=0, textColorRGB=[0, 0, 0])
        
def main():
    my_world = World(50, 50, TIME_STEP)

    my_world.set_gradient_source(GRADIENT_SOURCE)

    Agent.set_weights([ANGLE_WEIGHT, STRAIN_WEIGHT, GRADIENT_WEIGHT, REPULSION_WEIGHT])

    # set initial object positions

    angles = [None, 180, 180, 180, None]
    
    initial_robot_positions = get_starting_positions(UNSTRETCHED_TETHER_LENGTH, N, angles, [-7,-19,0])
    
    # Goal angles for each agent
    goal_angles = [None, 180, 180, 180, None]
    
    # initial_robot_positions = get_starting_positions(UNSTRETCHED_TETHER_LENGTH, N, goal_angles, [0,-1,0])

    # populates the list of robot objects with robot objects
    for i in range(N):
        my_world.create_agent(initial_robot_positions[i], 0, radius = RADIUS, goal_delta = goal_angles[i], height=HEIGHT, color=(1, 0, 0, 1))

    # populates the list of tether objects with tether objects
    for i in range(N-1):
        my_world.create_and_anchor_tether(my_world.agent_list[i], my_world.agent_list[i+1], UNSTRETCHED_TETHER_LENGTH, num_segments = 2)
        
    generate_circular_obstacle_course(my_world, 20, (0.1, 2), 20)

    add_axis_labels()
    
    runs = 0

    agent_to_update_next = 0

    shuffled_list = random.sample(my_world.agent_list, k=len(my_world.agent_list))

    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)

        for agent in shuffled_list:
            agent.sense_gradient(my_world.gradient_source)
            agent.sense_close_range(my_world.obj_list, sensing_mode=2)

        if runs % SENSING_PERIOD == 0:
            for i in range(len(shuffled_list)):
                if i == agent_to_update_next:
                    shuffled_list[i].compute_next_step()
                
            agent_to_update_next = agent_to_update_next + 1

            if agent_to_update_next >= len(shuffled_list):
                agent_to_update_next = 0

        runs = runs + 1
        
        p.stepSimulation()

if __name__ == "__main__":
    main()
