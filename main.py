"""
main.py

This file contains the main driver program for running the tetherbot PyBullet simulation.
"""

import pybullet as p
import pybullet_data
from world import World
from agent import Agent
import numpy as np

GRAVITY_Z = -9.81
HEIGHT = 0.01
TIME_STEP = 1./240.
N = 3
UNSTRETCHED_TETHER_LENGTH = 1
RADIUS = 0.1
GRADIENT_SOURCE = [-3,-3]
ANGLE_WEIGHT = 10
STRAIN_WEIGHT = 0
GRADIENT_WEIGHT = 0
REPULSION_WEIGHT = 0

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
        pos = [0, y[i], HEIGHT]
        positions.append(pos)

    return positions

def add_gui_labels():
    p.addUserDebugLine([0, 0, 0], [1, 0, 0], lineColorRGB=[0, 0, 1], lineWidth=10, lifeTime=0)
    p.addUserDebugLine([0, 0, 0], [0, 1, 0], lineColorRGB=[1, 0, 0], lineWidth=10, lifeTime=0)
    p.addUserDebugLine([0, 0, 0], [0, 0, 1], lineColorRGB=[.68, .12, .94], lineWidth=10, lifeTime=0)
    p.addUserDebugText("+x", [1, 0, 0], lifeTime=0, textColorRGB=[0, 0, 0])
    p.addUserDebugText("+y", [0, 1, 0], lifeTime=0, textColorRGB=[0, 0, 0])
    p.addUserDebugText("+z", [0, 0, 1], lifeTime=0, textColorRGB=[0, 0, 0])
        
def main():

    my_world = World(20, 20, TIME_STEP)

    my_world.set_gradient_source(GRADIENT_SOURCE)

    Agent.set_weights([ANGLE_WEIGHT, STRAIN_WEIGHT, GRADIENT_WEIGHT, REPULSION_WEIGHT])

    # set initial object positions
    initial_robot_positions = [[0, 0, HEIGHT],
                               [0, 1, HEIGHT],
                               [1, 1, HEIGHT]]
    
    # Goal angles for each agent
    
    # # list of x-y current target positions for each agent (starts at their initial positions)
    # target_pos = [initial_robot_positions[i][:2] for i in range(len(initial_robot_positions))]

    # # a list of all of the agent objects created
    # robot_ids = []

    # # a list of tether objects
    # tether_ids = []

    # populates the list of robot objects with robot objects
    for i in range(N):
        my_world.create_agent(initial_robot_positions[i], 0, radius = RADIUS, goal_delta = goal_angles[i])

    # populates the list of tether objects with tether objects
    for i in range(N-1):
        my_world.create_and_anchor_tether(my_world.agent_list[i], my_world.agent_list[i+1], UNSTRETCHED_TETHER_LENGTH, num_segments = 1)
        
    add_gui_labels()
    
    runs = 0

    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)

        # if runs%100 == 0:
        #     # calculate tether angle relative to each robot's heading
        #     sigma1 = get_sigma(robot_ids[1], tether_ids[0], tether_ids[1])
        #     # theta2 = get_theta(robot_ids[1], tether_ids[0])

        #     # display results in the GUI
        #     p.addUserDebugText(f"sigma = {sigma1:.2f} deg \n",
        #                         [0, 0.5, 0.5], textColorRGB=[0, 0, 0], lifeTime=1)
        

        # # calculate tether length and strain on every step
        # l = get_tether_length(tether_ids[0])
        # strain = (l - l_0) / l_0

        # # calculate tether angle relative to each robot's heading
        # sigma1 = get_sigma(robot_ids[1], tether_ids[0], tether_ids[1])
        # # theta2 = get_theta(robot_ids[1], tether_ids[0])

        # # calculates the direction the robot should move in to achieve the 
        # # goal delta
        # new_vector = new_position_for_sigma_goal(robot_ids[1], tether_ids[0], tether_ids[1], 270)

        # # display results in the GUI
        # p.addUserDebugText(f"tether length = {l:.2f} m\n tether strain = {strain:.2f}\n "
        #                     f"sigma = {sigma1:.2f} deg \n x_comp = {new_vector[0]:.2f} \n y_comp ={new_vector[1]:.2f} \n",
        #                     [0, 0.5, 0.5], textColorRGB=[0, 0, 0], lifeTime=1)

        my_world.agent_list[0].sense_gradient(my_world.gradient_source)
        my_world.agent_list[0].sense_close_range(my_world.obj_list)

        my_world.agent_list[1].sense_gradient(my_world.gradient_source)
        my_world.agent_list[1].sense_close_range(my_world.obj_list)

        my_world.agent_list[2].sense_gradient(my_world.gradient_source)
        my_world.agent_list[2].sense_close_range(my_world.obj_list)
        

        if my_world.agent_list[1].reached_target_position():
            my_world.agent_list[1].compute_next_step()
            my_world.agent_list[1].move_to()

        if my_world.agent_list[0].reached_target_position():
            my_world.agent_list[0].compute_next_step()
            my_world.agent_list[0].move_to()

        if my_world.agent_list[2].reached_target_position():
            my_world.agent_list[2].compute_next_step()
            my_world.agent_list[2].move_to()

        p.stepSimulation()

if __name__ == "__main__":
    main()
