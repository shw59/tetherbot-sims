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
ANGLE_WEIGHT = 2.5 # weighting of the angle vector in the overall resulting vector, normally 2.5
STRAIN_WEIGHT = 300 # weighting of the angle vector in the overall resulting vector, normally 300
GRADIENT_WEIGHT = 0.6 # weighting of the angle vector in the overall resulting vector, normally 0.6
REPULSION_WEIGHT = 1 # weighting of the angle vector in the overall resulting vector, normally 1
SENSING_PERIOD = 15 # The number of while loop iterations that should run before a singular, random
                   # agent updates its goal position

def get_starting_positions(l_0, n, angles, starting_position, direction):
    """
    Calculates and returns a list the starting position based on the list of 
    desired starting angles and the unstretched length of the tether between 
    the agents. Builds the agents in the direction specified by the input parameter,
    of which there are four allowable directions.
    starting_position: the position of the first agent, given as [x, y, z].
    direction: a string, can be "+x", "-x", "+y", or "-y", and will build the robots
               in the direction that is described in the string from the starting_position

    """
    the_list = []

    for i in range(n):
        the_list.append([0])

    the_list[0] = starting_position

    for i in range(1, n):
        if angles[i-1] == None:
            if direction == "+x":
                next_position = [the_list[i-1][0] + 1, the_list[i-1][1], the_list[i-1][2]]
            elif direction == "-x":
                next_position = [the_list[i-1][0] - 1, the_list[i-1][1], the_list[i-1][2]]
            elif direction == "+y":
                next_position = [the_list[i-1][0], the_list[i-1][1] + 1, the_list[i-1][2]]
            elif direction == "-y":
                next_position = [the_list[i-1][0], the_list[i-1][1] - 1, the_list[i-1][2]]
            else:
                print("Please specify the direction you want to build the robots. Can be '+x', '-x', '+y', or '-y'")
                return the_list
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
    """
    Adds x, y, and z axis labels so that it is easier to orient oneself
    """
    p.addUserDebugLine([0, 0, 0], [1, 0, 0], lineColorRGB=[0, 0, 1], lineWidth=10, lifeTime=0)
    p.addUserDebugLine([0, 0, 0], [0, 1, 0], lineColorRGB=[1, 0, 0], lineWidth=10, lifeTime=0)
    p.addUserDebugLine([0, 0, 0], [0, 0, 1], lineColorRGB=[.68, .12, .94], lineWidth=10, lifeTime=0)
    p.addUserDebugText("+x", [1, 0, 0], lifeTime=0, textColorRGB=[0, 0, 0])
    p.addUserDebugText("+y", [0, 1, 0], lifeTime=0, textColorRGB=[0, 0, 0])
    p.addUserDebugText("+z", [0, 0, 1], lifeTime=0, textColorRGB=[0, 0, 0])

def storm_drain():
    """
    Generates an environment like the one described in the research paper. The robots go through a pipe, get
    to a storm drain, and then collect debris in the storm drain and bring it to the top of the tank.
    """
    n = 9

    a_weight = 0 # angle vector weighting
    s_weight = 100 # strain vector weighting
    g_weight = 50 # gradient vector weighting
    r_weight = 4 # repulsion vector weighting

    gradient = [0,0]

    my_world = World(100, 100, TIME_STEP)

    my_world.set_gradient_source(gradient)

    Agent.set_weights([a_weight, s_weight, g_weight, r_weight])

    angles = [None, 180, 180, 180, 180, 180, 180, 180, None]
    
    initial_robot_positions = get_starting_positions(UNSTRETCHED_TETHER_LENGTH, n, angles, [-30,-7,0], "+x")
    
    goal_angles = [None, 180, 180, 180, 180, 180, 180, 180, None]

    # populates the list of robot objects with robot objects
    for i in range(n):
        my_world.create_agent(initial_robot_positions[i], 0, radius = RADIUS, goal_delta = goal_angles[i], height=HEIGHT, color=(1, 0, 0, 1))

    # populates the list of tether objects with tether objects
    for i in range(n-1):
        my_world.create_and_anchor_tether(my_world.agent_list[i], my_world.agent_list[i+1], UNSTRETCHED_TETHER_LENGTH, num_segments = 10)

    # sides of sewer tank

    for i in range(30):
        my_world.create_obstacle("cube", [-10, i], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
        my_world.create_obstacle("cube", [10, i], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)

    # top of the sewer tank
    my_world.create_obstacle("cube", [-10, 30], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-9, 30], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-8, 30], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-7, 30], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-6, 30], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-5, 30], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-4, 30], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [4, 30], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [5, 30], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [6, 30], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [7, 30], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [8, 30], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [9, 30], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [10, 30], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)


    # bottom of the sewer tank
    my_world.create_obstacle("cube", [-9, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-8, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-7, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-6, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-5, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-4, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-3, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-2, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-1, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [0, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [1, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [2, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [3, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [4, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [5, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [6, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [7, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [8, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [9, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [10, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [10, -1], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)

    # tube connected to the tank
    my_world.create_obstacle("cube", [-10, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-11, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-12, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-13, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-14, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-10, 0], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-11, 0], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-12, 0], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-13, 0], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-14, 0], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-15, 0], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)


    # diagonal part
    my_world.create_obstacle("cube", [-15, -3], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-16, -4], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-17, -5], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-18, -6], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-19, -7], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-20, -8], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-21, -8], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-16, -1], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-17, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-18, -3], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-19, -4], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-20, -5], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-21, -6], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)

    # tube where the agents start
    my_world.create_obstacle("cube", [-22, -6], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-23, -6], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-24, -6], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-25, -6], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-26, -6], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-27, -6], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-28, -6], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-29, -6], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-30, -6], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-22, -8], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-23, -8], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-24, -8], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-25, -8], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-26, -8], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-27, -8], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-28, -8], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-29, -8], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)
    my_world.create_obstacle("cube", [-30, -8], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.25)

    number_of_fixed_obstacles = 0

    for i in range(number_of_fixed_obstacles):
        x = random.uniform(-9,9)
        y = random.uniform(-1, 29)
        sizes = random.uniform(0.5,1)
        my_world.create_obstacle("hexagon", [x, y], length=sizes, width=sizes, color=(1, 0, 1, 1), fixed=True, height=0.25)

    number_of_non_fixed_obstacles = 200

    for i in range(number_of_non_fixed_obstacles):
        x = random.uniform(-9,9)
        y = random.uniform(1, 29)
        size = random.uniform(0.1,0.25)
        my_world.create_obstacle("hexagon", [x, y], length=size, width=size, color=(1, 0, 1, 1), fixed=False, height=0.25, mass=0.01)

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

def basic_test():
    """
    Generates a very simple formation of agents in order to test the hysteresis.
    """
    n = 3

    a_weight = 30 # angle vector weighting
    s_weight = 300 # strain vector weighting
    g_weight = 0.6 # gradient vector weighting
    r_weight = 1 # repulsion vector weighting

    gradient = [0,0]

    my_world = World(15, 15, TIME_STEP)

    my_world.set_gradient_source(gradient)

    Agent.set_weights([a_weight, s_weight, g_weight, r_weight])

    angles = [None, 90, None]
    
    initial_agent_positions = get_starting_positions(UNSTRETCHED_TETHER_LENGTH, n, angles, [-1,0,0], "+x")
    
    goal_angles = [None, 270, None]

    # populates the list of robot objects with agent objects
    for i in range(n):
        my_world.create_agent(initial_agent_positions[i], 0, radius = RADIUS, goal_delta = goal_angles[i], height=HEIGHT, color=(1, 0, 0, 1))

    # populates the list of tether objects with tether objects
    for i in range(n-1):
        my_world.create_and_anchor_tether(my_world.agent_list[i], my_world.agent_list[i+1], UNSTRETCHED_TETHER_LENGTH, num_segments = 5)

    number_of_fixed_obstacles = 0

    for i in range(number_of_fixed_obstacles):
        x = random.uniform(-9,9)
        y = random.uniform(-1, 29)
        sizes = random.uniform(0.5,1)
        my_world.create_obstacle("hexagon", [x, y], length=sizes, width=sizes, color=(1, 0, 1, 1), fixed=True, height=0.25)

    number_of_non_fixed_obstacles = 0

    for i in range(number_of_non_fixed_obstacles):
        x = random.uniform(-9,9)
        y = random.uniform(1, 29)
        size = random.uniform(0.1,0.25)
        my_world.create_obstacle("hexagon", [x, y], length=size, width=size, color=(1, 0, 1, 1), fixed=False, height=0.25, mass=0.01)

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
        
def main():
    """
    Is the function called when running the program. This function calls which ever function you want to test.
    """
    basic_test()

if __name__ == "__main__":
    main()
