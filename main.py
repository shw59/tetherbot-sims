"""
main.py

This file contains the main driver program for running the tetherbot PyBullet simulation.
"""

import pybullet as p
import pybullet_data
from world import World
from agent import Agent
from simulations import Simulation
import numpy as np
import math
import random
import csv
import time

TIME_STEP = 1/240 # seconds
SENSING_PERIOD = 5 # number of while loop iterations that run before an agent position updates
LOGGING_PERIOD = 20 # number of while loop iterations that pass before data is written to a csv file

# jackal robot parameters
MASS = 17 # kg
RADIUS = 0.29 # m
HEIGHT = 0.25 # m
MAX_SPEED = 2 # m/s
DRIVE_POWER = 500 # watts
MU_STATIC = 1.25
MU_DYNAMIC = 0.9

# paracord 550 parameters
UNSTRETCHED_TETHER_LENGTH = 2
YOUNGS_MODULUS = 650e6
DIAMETER = 0.0019 # m

N = 3

GRADIENT_SOURCE = [0, 0]
ANGLE_WEIGHT = 2.5 # weighting of the angle vector in the overall resulting vector, normally 2.5
STRAIN_WEIGHT = 300 # weighting of the angle vector in the overall resulting vector, normally 300
GRADIENT_WEIGHT = 0.6 # weighting of the angle vector in the overall resulting vector, normally 0.6
REPULSION_WEIGHT = 1 # weighting of the angle vector in the overall resulting vector, normally 1

def basic_starting_positions(l_0, n, angles, starting_position, direction):
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
                next_position = [the_list[i-1][0] + UNSTRETCHED_TETHER_LENGTH, the_list[i-1][1], the_list[i-1][2]]
            elif direction == "-x":
                next_position = [the_list[i-1][0] - UNSTRETCHED_TETHER_LENGTH, the_list[i-1][1], the_list[i-1][2]]
            elif direction == "+y":
                next_position = [the_list[i-1][0], the_list[i-1][1] + UNSTRETCHED_TETHER_LENGTH, the_list[i-1][2]]
            elif direction == "-y":
                next_position = [the_list[i-1][0], the_list[i-1][1] - UNSTRETCHED_TETHER_LENGTH, the_list[i-1][2]]
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

def display_axis_labels():
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
    
    initial_robot_positions = basic_starting_positions(UNSTRETCHED_TETHER_LENGTH, n, angles, [-30,-7,0], "+x")
    
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

    display_axis_labels()
    
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
                    shuffled_list[i].set_next_step()
                
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


    a_weight = 10 # angle vector weighting
    s_weight = 15 # strain vector weighting
    g_weight = 2 # gradient vector weighting
    r_weight = 3 # repulsion vector weighting


    gradient = [0,-10]

    my_world = World(15, 15, TIME_STEP)

    my_world.set_gradient_source(gradient)

    Agent.set_weights([a_weight, s_weight, g_weight, r_weight])

    angles = [None, 90, None]
    
    initial_agent_positions = basic_starting_positions(UNSTRETCHED_TETHER_LENGTH, n, angles, [-1,0,0], "+x")
    
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

    display_axis_labels()
    
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
                    shuffled_list[i].set_next_step()
              
            strain_m = my_world.agent_list[1].tethers[0].get_strain()
            strain_p = my_world.agent_list[1].tethers[1].get_strain()

            x_velocity = p.getJointState(my_world.agent_list[1].id, 1)[1]
            y_velocity = p.getJointState(my_world.agent_list[1].id, 0)[1]

            total_velocity = math.sqrt(x_velocity**2 + y_velocity**2)

            p.addUserDebugText(f"tether_m strain = {strain_m:.2f} tether_p strain = {strain_p:.2f}, velocity = {total_velocity:.2f}",
                               [0, 0.5, 0.5], textColorRGB=[0, 0, 0], lifeTime=1)

            agent_to_update_next = agent_to_update_next + 1

            if agent_to_update_next >= len(shuffled_list):
                agent_to_update_next = 0

        runs = runs + 1
        
        p.stepSimulation()
        
def angle_and_position_offset(n, angle, offset, seperation):
    """
    Returns 2D positions that are in a straight line that is some angle off of the 
    y-axis from the y-intercept position, and the y-intercept is determined based
    on the offset.
    n: number of positions to generate
    angle: angle off of +y-axis towards +x-axis in degrees
    offset: value of the y-offset of the center of the line of agents about (0,0)
    """

    positions = []

    for i in range(n):
        positions.append([0,0,0])

    bottom = 0.5*seperation*(1-n)
    positions[0] = [bottom, 0]

    for i in range(1, n):
        positions[i] = [bottom+seperation*(i), 0, 0]

    angle_off_positive_x = np.radians(90 - angle)

    for i in range(n):
        positions[i] = [round(np.cos(angle_off_positive_x)*positions[i][0], 5), round(np.sin(angle_off_positive_x)*positions[i][0], 5) + offset, 0]

    return positions

def log_to_csv(filename, data_row, header=None):
    """
    Logs a data row to a CSV file.
    If header is provided and the file is new, it writes the header first.
    """
    with open(filename, 'a', newline='') as csvfile:
        writer = csv.writer(csvfile)

        # Check if the file is empty to write header
        if csvfile.tell() == 0 and header:
            writer.writerow(header)

        writer.writerow(data_row)

def obstacle_avoidance(n, l_0, y_offset, angle_off_y, a_weight = 10, s_weight = 15, g_weight = 10, r_weight = 3, gradient = [20,0], obst_pos = [10,0], obst_radius = 1, obst_height = 1, obst_type = "hexagon", stop=2000, trial = 0):
    """
    Generates a very simple formation of agents in order to test the hysteresis.
    """

    my_world = World(200, 200, TIME_STEP)

    my_world.set_gradient_source(gradient)

    Agent.set_weights([a_weight, s_weight, g_weight, r_weight])
    
    initial_agent_positions = angle_and_position_offset(n, angle_off_y, y_offset, l_0)

    goals = []
    for i in range(n):
        if i == 0 or i == (n-1):
            goals.append(None)
        else:
            goals.append(180)

    # populates the list of robot objects with agent objects
    for i in range(n):
        my_world.create_agent(initial_agent_positions[i], 0, radius = RADIUS, goal_delta = goals[i], height=HEIGHT, color=(1, 0, 0, 1))

    # populates the list of tether objects with tether objects
    for i in range(n-1):
        my_world.create_and_anchor_tether(my_world.agent_list[i], my_world.agent_list[i+1], l_0, num_segments = 5)

    my_world.create_obstacle(obst_type, obst_pos, length=obst_radius, width=obst_radius, color=(1, 0, 1, 1), fixed=True, height=obst_height)

    display_axis_labels()
    
    runs = 0

    agent_to_update_next = 0

    shuffled_list = random.sample(my_world.agent_list, k=len(my_world.agent_list))

    # log_file = str(angle_off_y) + "_degree_" + str(y_offset) + "_offset_.csv"
    log_file = "trial: " + str(trial) + ", degree: " + str(angle_off_y) + ", offset: " + str(y_offset) + ".csv"

    log_header = ['Timestep']

    for i in range(n):
        log_header.append('agent_' + str(i) + '_x')
        log_header.append('agent_' + str(i) + '_y')

    
    # main simulation loop
    while (runs <= stop) and (p.isConnected()):
        if runs%30:
            p.getCameraImage(320,200)

        for agent in shuffled_list:
            agent.sense_gradient(my_world.gradient_source)
            agent.sense_close_range(my_world.obj_list, sensing_mode=2)

        if runs % SENSING_PERIOD == 0:
            for i in range(len(shuffled_list)):
                if i == agent_to_update_next:
                    shuffled_list[i].set_next_step()
            
            # strain_m = my_world.agent_list[1].tethers[0].get_strain()
            # strain_p = my_world.agent_list[1].tethers[1].get_strain()

            # x_velocity = p.getJointState(my_world.agent_list[1].id, 1)[1]
            # y_velocity = p.getJointState(my_world.agent_list[1].id, 0)[1]

            # total_velocity = math.sqrt(x_velocity**2 + y_velocity**2)

            # p.addUserDebugText(f"tether_m strain = {strain_m:.2f} tether_p strain = {strain_p:.2f}, velocity = {total_velocity:.2f}",
            #                    [0, 0.5, 0.5], textColorRGB=[0, 0, 0], lifeTime=1)
            
            agent_to_update_next = agent_to_update_next + 1

            if agent_to_update_next >= len(shuffled_list):
                agent_to_update_next = 0

        if runs%LOGGING_PERIOD == 0:
            data = [runs]
            for agent in my_world.agent_list:
                data.append(round(agent.get_pose()[0][0], 5))
                data.append(round(agent.get_pose()[0][1], 5))
        
            log_to_csv(log_file, data, header=log_header)

        runs = runs + 1
        
        p.stepSimulation()

    p.disconnect()

    return log_file

def obstacle_avoidance_success(list_of_files, number_of_trials, number_of_runs_per_trial, number_of_while_runs):

    if (number_of_while_runs-4*LOGGING_PERIOD <= 0):
        print("Increase the number of while loop iterations please")

        return 0
    
    else:

        each_runs_trial = []

        for i in range(number_of_runs_per_trial):
            set_up = []
            for k in range(number_of_trials):
                set_up.append(list_of_files[i+k*number_of_runs_per_trial])
            each_runs_trial.append(set_up)

        success_list = []

        for i in range(number_of_trials):
            success_list.append(0)


        for i in range(len(each_runs_trial)):
            total = 0
            successful = 0
            for k in range(number_of_trials):
                with open(each_runs_trial[i][k]) as csv_file:
                    csv_reader = csv.reader(csv_file, delimiter=',')
                    initial_x = 0
                    initial_y = 0
                    final_x = 0
                    final_y = 0
                    for row in csv_reader:
                    
                        if row[0] == (number_of_while_runs*(0.9)):
                            initial_x = row[1]
                            initial_y = row[2]

                        if row[0] == (number_of_while_runs):
                            final_x = row[1]
                            final_y = row[2]

                    print(initial_x)
                    print(initial_y)
                    print(final_x)
                    print(final_y)
                    if (abs(final_x - initial_x) <=1 and abs(final_y - initial_y) <=1):
                        total = total + 1
                    else:
                        successful = successful + 1
                        total = total + 1

            success_list[i] = successful/total

        print(success_list)
        
        return 0

def run_obstacle_simulations(n, l_0, length_of_simulation, offsets, angles_to_try, number_of_trials):
    """
    length_of_simulation: this number is an integer, and it is multiplied by the 
                          LOGGING_PERIOD to determine how long to run the while loop for

    """
    list_of_file_names = []
    for t in range(number_of_trials):
        for o in offsets:
            for a in angles_to_try:
                list_of_file_names.append(obstacle_avoidance(n, l_0, o, a, stop = length_of_simulation*LOGGING_PERIOD, trial = t + 1, obst_radius=4*l_0))

    obstacle_avoidance_success(list_of_files=list_of_file_names, number_of_trials=number_of_trials, number_of_runs_per_trial = len(offsets) * len(angles_to_try), number_of_while_runs=length_of_simulation*LOGGING_PERIOD)



def main():
    """
    Is the function called when running the program. This function calls which ever function you want to test.
    """
    sim = Simulation(TIME_STEP, MASS, RADIUS, HEIGHT, MAX_SPEED, DRIVE_POWER, MU_STATIC, MU_DYNAMIC, 
                     UNSTRETCHED_TETHER_LENGTH, YOUNGS_MODULUS, DIAMETER, SENSING_PERIOD, LOGGING_PERIOD, gui_on=True)
    
    sim.run_tow_failed_agents_simulations(5, 10, [0, 1, 2, 3, 4])
    sim.run_object_capture_simulations(9, 10, [5, 10, 30, 50], maintain_line=False)
    sim.run_object_capture_simulations(9, 10, [5, 10, 30, 50], maintain_line=True)

if __name__ == "__main__":
    main()
