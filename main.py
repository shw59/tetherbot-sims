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

HEIGHT = 0.01
TIME_STEP = 1./240.
N = 60
UNSTRETCHED_TETHER_LENGTH = 1
RADIUS = 0.1
GRADIENT_SOURCE = [0,0]
ANGLE_WEIGHT = 2
STRAIN_WEIGHT = 100
GRADIENT_WEIGHT = 2
REPULSION_WEIGHT = 4

def set_straight_line(n, spacing):
    """
    Returns a list of positions that correspond to n robots seperated by "spacing" distance
    along the x-axis, centered about zero.
    """
    positions = []
    if (n%2 == 0):
        left = n/2
        right = n - left
        y = np.linspace(-left*spacing, right*spacing, n+1)
    else:
        left = round(n/2)
        right = n - left
        y = np.linspace(-left*spacing, right*spacing, n+1)
    for i in range(n):
        pos = [0, y[i], 0]
        positions.append(pos)

    return positions

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
            next_position = [the_list[i-1][0], the_list[i-1][1] + 1, the_list[i-1][2]]
        else:
            prev_theta = np.degrees(math.atan2(the_list[i-2][1] - the_list[i-1][1], the_list[i-2][0] - the_list[i-1][0]))
            new_theta = prev_theta + (360-angles[i-1])
            new_x = the_list[i-1][0] + l_0*np.cos(np.radians(new_theta))
            new_y = the_list[i-1][1] + l_0*np.sin(np.radians(new_theta))
            next_position = [new_x, new_y, 0]

        the_list[i] = next_position


    return the_list

def add_axis_labels():
    p.addUserDebugLine([0, 0, 0], [1, 0, 0], lineColorRGB=[0, 0, 1], lineWidth=10, lifeTime=0)
    p.addUserDebugLine([0, 0, 0], [0, 1, 0], lineColorRGB=[1, 0, 0], lineWidth=10, lifeTime=0)
    p.addUserDebugLine([0, 0, 0], [0, 0, 1], lineColorRGB=[.68, .12, .94], lineWidth=10, lifeTime=0)
    p.addUserDebugText("+x", [1, 0, 0], lifeTime=0, textColorRGB=[0, 0, 0])
    p.addUserDebugText("+y", [0, 1, 0], lifeTime=0, textColorRGB=[0, 0, 0])
    p.addUserDebugText("+z", [0, 0, 1], lifeTime=0, textColorRGB=[0, 0, 0])
        
def main():

    my_world = World(200, 200, TIME_STEP)

    my_world.set_gradient_source(GRADIENT_SOURCE)

    Agent.set_weights([ANGLE_WEIGHT, STRAIN_WEIGHT, GRADIENT_WEIGHT, REPULSION_WEIGHT])

    # set initial object positions

    angles = [None, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180,
              180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180,
              180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180,
              180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, None]
    
    initial_robot_positions = get_starting_positions(UNSTRETCHED_TETHER_LENGTH, N, angles, [0,-30,0])
    
    # Goal angles for each agent
    goal_angles = [None, 180, 180, 180, 180, 180, 180, 180, 180, 180, 225, 180, 180, 225, 180, 180, 180, 180, 
                   225, 180, 180, 225, 180, 270, 180, 180, 270, 90, 180, 180, 90,
                   180, 180, 180, 180, 180, 180, 180, 180, 90, 180, 180, 90, 
                   270, 180, 180, 270, 180, 225, 180, 180, 225, 180, 180, 180, 180, 225, 180, 180, None]
    

    # populates the list of robot objects with robot objects
    for i in range(N):
        my_world.create_agent(initial_robot_positions[i], 0, radius = RADIUS, goal_delta = goal_angles[i], height=HEIGHT, color=(1, 0, 0, 1))

    # populates the list of tether objects with tether objects
    for i in range(N-1):
        my_world.create_and_anchor_tether(my_world.agent_list[i], my_world.agent_list[i+1], UNSTRETCHED_TETHER_LENGTH, num_segments = 2)
        
    add_axis_labels()
    
    runs = 0

    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)

        for agent in my_world.agent_list:
            agent.sense_gradient(my_world.gradient_source)
            agent.sense_close_range(my_world.obj_list)

        for agent in my_world.agent_list:
            if runs % 5 == 0:
                agent.compute_next_step()
                agent.move_to()
            if agent.reached_target_position():
                agent.compute_next_step()
                agent.move_to()
        
        p.stepSimulation()

if __name__ == "__main__":
    main()
