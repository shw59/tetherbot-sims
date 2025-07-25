"""
unit_tests.py

This file defines a suite of unit tests in PyBullet for tetherbot simulation functionality.
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
ANGLE_WEIGHT = 1
STRAIN_WEIGHT = 10
GRADIENT_WEIGHT = 0
REPULSION_WEIGHT = 0

def set_straight_line(n, spacing):
    """
    Returns a list of positions that correspond to n robots seperated by "spacing" distance
    along the x-axis, centered about zero.
    """
    positions = []
    if (n % 2 == 0):
        left = n / 2
        right = n - left
        y = np.linspace(-left * spacing, right * spacing, n + 1)
    else:
        left = round(n / 2)
        right = n - left
        y = np.linspace(-left * spacing, right * spacing, n + 1)
    for i in range(n):
        pos = [0, y[i], HEIGHT]
        positions.append(pos)

    return positions

def display_axis_labels():
    p.addUserDebugLine([0, 0, 0], [1, 0, 0], lineColorRGB=[0, 0, 1], lineWidth=10, lifeTime=0)
    p.addUserDebugLine([0, 0, 0], [0, 1, 0], lineColorRGB=[1, 0, 0], lineWidth=10, lifeTime=0)
    p.addUserDebugLine([0, 0, 0], [0, 0, 1], lineColorRGB=[.68, .12, .94], lineWidth=10, lifeTime=0)
    p.addUserDebugText("+x", [1, 0, 0], lifeTime=0, textColorRGB=[0, 0, 0])
    p.addUserDebugText("+y", [0, 1, 0], lifeTime=0, textColorRGB=[0, 0, 0])
    p.addUserDebugText("+z", [0, 0, 1], lifeTime=0, textColorRGB=[0, 0, 0])

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

# The following tests all relate to the calculation of the angle vector

# Successful!
def test_delta_calculation():
    """
    Tests to see if the delta calculation is correct
    """
     
    my_world = World(20, 20, TIME_STEP)

    my_world.set_gradient_source(GRADIENT_SOURCE)

    Agent.set_weights([10, 0, 0, 0])

    # set initial object positions
    initial_robot_positions = [[0, 0, HEIGHT],
                               [0, 1, HEIGHT],
                               [1, 1, HEIGHT]]
    
    # Goal angles for each agent
    goal_angles = [None, 90, None]

    # populates the list of robot objects with robot objects
    for i in range(N):
        my_world.create_agent(initial_robot_positions[i], 0, radius = RADIUS, goal_delta = goal_angles[i])

    # populates the list of tether objects with tether objects
    for i in range(N-1):
        my_world.create_and_anchor_tether(my_world.agent_list[i], my_world.agent_list[i+1], UNSTRETCHED_TETHER_LENGTH, num_segments = 1)

    display_axis_labels()

    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)

        # calculate tether angle relative to each robot's heading
        delta = my_world.agent_list[1].get_delta()
        # theta2 = get_theta(robot_ids[1], tether_ids[0])

        # display results in the GUI
        p.addUserDebugText(f"delta = {delta:.2f} deg \n",
                            [0, 0.5, 0.5], textColorRGB=[0, 0, 0], lifeTime=1)
        
        p.stepSimulation()

# Successful!
def test_angle_vector_270_to_90():
    """
    Tests to see if the angle vector is properly computed and
    agents move in a direction that attempts to fulfill their agnle goal.
    """
     
    my_world = World(20, 20, TIME_STEP)

    my_world.set_gradient_source(GRADIENT_SOURCE)

    Agent.set_weights([10, 0, 0, 0])

    # set initial object positions
    initial_robot_positions = [[0, 0, HEIGHT],
                               [0, 1, HEIGHT],
                               [1, 1, HEIGHT]]
    
    # Goal angles for each agent
    goal_angles = [None, 90, None]

    # populates the list of robot objects with robot objects
    for i in range(N):
        my_world.create_agent(initial_robot_positions[i], 0, radius = RADIUS, goal_delta = goal_angles[i])

    # populates the list of tether objects with tether objects
    for i in range(N-1):
        my_world.create_and_anchor_tether(my_world.agent_list[i], my_world.agent_list[i+1], UNSTRETCHED_TETHER_LENGTH, num_segments = 1)

    display_axis_labels()

    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)
        
        if my_world.agent_list[1].reached_target_position():
            my_world.agent_list[1].set_next_step()
        
        p.stepSimulation()

# Successful!
def test_angle_vector_90_to_270():
    """
    Tests to see if the angle vector is properly computed and
    agents move in a direction that attempts to fulfill their agnle goal.
    """
     
    my_world = World(20, 20, TIME_STEP)

    my_world.set_gradient_source(GRADIENT_SOURCE)

    Agent.set_weights([10, 0, 0, 0])

    # set initial object positions
    initial_robot_positions = [[0, 0, HEIGHT],
                               [1, 0, HEIGHT],
                               [1, 1, HEIGHT]]
    
    # Goal angles for each agent
    goal_angles = [None, 270, None]

    # populates the list of robot objects with robot objects
    for i in range(N):
        my_world.create_agent(initial_robot_positions[i], 0, radius = RADIUS, goal_delta = goal_angles[i])

    # populates the list of tether objects with tether objects
    for i in range(N-1):
        my_world.create_and_anchor_tether(my_world.agent_list[i], my_world.agent_list[i+1], UNSTRETCHED_TETHER_LENGTH, num_segments = 1)

    display_axis_labels()

    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)
        
        if my_world.agent_list[1].reached_target_position():
            my_world.agent_list[1].set_next_step()
        
        p.stepSimulation()

# Successful!
def test_angle_vector_90_to_90():
    """
    Tests to see if the angle vector is properly computed and
    agents move in a direction that attempts to fulfill their agnle goal.
    """
     
    my_world = World(20, 20, TIME_STEP)

    my_world.set_gradient_source(GRADIENT_SOURCE)

    Agent.set_weights([10, 0, 0, 0])

    # set initial object positions
    initial_robot_positions = [[0, 0, HEIGHT],
                               [1, 0, HEIGHT],
                               [1, 1, HEIGHT]]
    
    # Goal angles for each agent
    goal_angles = [None, 90, None]

    # populates the list of robot objects with robot objects
    for i in range(N):
        my_world.create_agent(initial_robot_positions[i], 0, radius = RADIUS, goal_delta = goal_angles[i])

    # populates the list of tether objects with tether objects
    for i in range(N-1):
        my_world.create_and_anchor_tether(my_world.agent_list[i], my_world.agent_list[i+1], UNSTRETCHED_TETHER_LENGTH, num_segments = 1)

    display_axis_labels()

    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)
        
        if my_world.agent_list[1].reached_target_position():
            my_world.agent_list[1].set_next_step()
        
        p.stepSimulation()

# Successful!
def test_angle_vector_270_to_270():
    """
    Tests to see if the angle vector is properly computed and
    agents move in a direction that attempts to fulfill their agnle goal.
    """
     
    my_world = World(20, 20, TIME_STEP)

    my_world.set_gradient_source(GRADIENT_SOURCE)

    Agent.set_weights([10, 0, 0, 0])

    # set initial object positions
    initial_robot_positions = [[0, 0, HEIGHT],
                               [0, 1, HEIGHT],
                               [1, 1, HEIGHT]]
    
    # Goal angles for each agent
    goal_angles = [None, 270, None]

    # populates the list of robot objects with robot objects
    for i in range(N):
        my_world.create_agent(initial_robot_positions[i], 0, radius = RADIUS, goal_delta = goal_angles[i])

    # populates the list of tether objects with tether objects
    for i in range(N-1):
        my_world.create_and_anchor_tether(my_world.agent_list[i], my_world.agent_list[i+1], UNSTRETCHED_TETHER_LENGTH, num_segments = 1)

    display_axis_labels()

    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)
        
        if my_world.agent_list[1].reached_target_position():
            my_world.agent_list[1].set_next_step()
        
        p.stepSimulation()

# Successful!
def test_angle_vector_90_to_180():
    """
    Tests to see if the angle vector is properly computed and
    agents move in a direction that attempts to fulfill their agnle goal.
    """
     
    my_world = World(20, 20, TIME_STEP)

    my_world.set_gradient_source(GRADIENT_SOURCE)

    Agent.set_weights([10, 0, 0, 0])

    # set initial object positions
    initial_robot_positions = [[0, 0, HEIGHT],
                               [1, 0, HEIGHT],
                               [1, 1, HEIGHT]]
    
    # Goal angles for each agent
    goal_angles = [None, 180, None]

    # populates the list of robot objects with robot objects
    for i in range(N):
        my_world.create_agent(initial_robot_positions[i], 0, radius = RADIUS, goal_delta = goal_angles[i])

    # populates the list of tether objects with tether objects
    for i in range(N-1):
        my_world.create_and_anchor_tether(my_world.agent_list[i], my_world.agent_list[i+1], UNSTRETCHED_TETHER_LENGTH, num_segments = 1)

    display_axis_labels()

    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)
        
        if my_world.agent_list[1].reached_target_position():
            my_world.agent_list[1].set_next_step()
        
        p.stepSimulation()

# Successful!
def test_angle_vector_270_to_180():
    """
    Tests to see if the angle vector is properly computed and
    agents move in a direction that attempts to fulfill their agnle goal.
    """
     
    my_world = World(20, 20, TIME_STEP)

    my_world.set_gradient_source(GRADIENT_SOURCE)

    Agent.set_weights([10, 0, 0, 0])

    # set initial object positions
    initial_robot_positions = [[0, 0, HEIGHT],
                               [0, 1, HEIGHT],
                               [1, 1, HEIGHT]]
    
    # Goal angles for each agent
    goal_angles = [None, 180, None]

    # populates the list of robot objects with robot objects
    for i in range(N):
        my_world.create_agent(initial_robot_positions[i], 0, radius = RADIUS, goal_delta = goal_angles[i])

    # populates the list of tether objects with tether objects
    for i in range(N-1):
        my_world.create_and_anchor_tether(my_world.agent_list[i], my_world.agent_list[i+1], UNSTRETCHED_TETHER_LENGTH, num_segments = 1)

    display_axis_labels()

    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)
        
        if my_world.agent_list[1].reached_target_position():
            my_world.agent_list[1].set_next_step()
        
        p.stepSimulation()

# Successful!
def test_angle_vector_180_to_270():
    """
    Tests to see if the angle vector is properly computed and
    agents move in a direction that attempts to fulfill their agnle goal.
    """
     
    my_world = World(20, 20, TIME_STEP)

    my_world.set_gradient_source(GRADIENT_SOURCE)

    Agent.set_weights([10, 0, 0, 0])

    # set initial object positions
    initial_robot_positions = [[-1, 0, HEIGHT],
                               [0, 0, HEIGHT],
                               [1, 0, HEIGHT]]
    
    # Goal angles for each agent
    goal_angles = [None, 270, None]

    # populates the list of robot objects with robot objects
    for i in range(N):
        my_world.create_agent(initial_robot_positions[i], 0, radius = RADIUS, goal_delta = goal_angles[i])

    # populates the list of tether objects with tether objects
    for i in range(N-1):
        my_world.create_and_anchor_tether(my_world.agent_list[i], my_world.agent_list[i+1], UNSTRETCHED_TETHER_LENGTH, num_segments = 1)

    display_axis_labels()

    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)
        
        if my_world.agent_list[1].reached_target_position():
            my_world.agent_list[1].set_next_step()
        
        p.stepSimulation()

# Successful!
def test_angle_vector_180_to_90():
    """
    Tests to see if the angle vector is properly computed and
    agents move in a direction that attempts to fulfill their agnle goal.
    """
     
    my_world = World(20, 20, TIME_STEP)

    my_world.set_gradient_source(GRADIENT_SOURCE)

    Agent.set_weights([10, 0, 0, 0])

    # set initial object positions
    initial_robot_positions = [[-1, 0, HEIGHT],
                               [0, 0, HEIGHT],
                               [1, 0, HEIGHT]]
    
    # Goal angles for each agent
    goal_angles = [None, 90, None]

    # populates the list of robot objects with robot objects
    for i in range(N):
        my_world.create_agent(initial_robot_positions[i], 0, radius = RADIUS, goal_delta = goal_angles[i])

    # populates the list of tether objects with tether objects
    for i in range(N-1):
        my_world.create_and_anchor_tether(my_world.agent_list[i], my_world.agent_list[i+1], UNSTRETCHED_TETHER_LENGTH, num_segments = 1)

    display_axis_labels()

    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)
        
        if my_world.agent_list[1].reached_target_position():
            my_world.agent_list[1].set_next_step()
        
        p.stepSimulation()

# Successful!
def test_angle_vector_180_to_180():
    """
    Tests to see if the angle vector is properly computed and
    agents move in a direction that attempts to fulfill their agnle goal.
    """
     
    my_world = World(20, 20, TIME_STEP)

    my_world.set_gradient_source(GRADIENT_SOURCE)

    Agent.set_weights([10, 0, 0, 0])

    # set initial object positions
    initial_robot_positions = [[-1, 0, HEIGHT],
                               [0, 0, HEIGHT],
                               [1, 0, HEIGHT]]
    
    # Goal angles for each agent
    goal_angles = [None, 180, None]

    # populates the list of robot objects with robot objects
    for i in range(N):
        my_world.create_agent(initial_robot_positions[i], 0, radius = RADIUS, goal_delta = goal_angles[i])

    # populates the list of tether objects with tether objects
    for i in range(N-1):
        my_world.create_and_anchor_tether(my_world.agent_list[i], my_world.agent_list[i+1], UNSTRETCHED_TETHER_LENGTH, num_segments = 1)

    display_axis_labels()

    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)
        
        if my_world.agent_list[1].reached_target_position():
            my_world.agent_list[1].set_next_step()
        
        p.stepSimulation()     
        
# The following tests all relate to the calculation of the strain vector

def test_strain():
    n = 2

    angle_weight = 0
    strain_weight = 10
    gradient_weight = 0
    repulsion_weight = 0

    my_world = World(20, 20, TIME_STEP)

    Agent.set_weights([angle_weight, strain_weight, gradient_weight, repulsion_weight])

    # set initial object positions
    initial_robot_positions = [[0, 0, HEIGHT],
                               [0, 1, HEIGHT]]

    # populates the list of robot objects with robot objects
    for i in range(n):
        my_world.create_agent(initial_robot_positions[i], 0, radius = RADIUS)

    # populates the list of tether objects with tether objects
    for i in range(n-1):
        my_world.create_and_anchor_tether(my_world.agent_list[i], my_world.agent_list[i+1], UNSTRETCHED_TETHER_LENGTH, num_segments=5)

    display_axis_labels()

    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)

        strain = 0
        for agent in my_world.agent_list:
            if agent.tethers[0]:
                strain = agent.tethers[0].get_strain()

        p.addUserDebugText(f"tether strain = {strain:.2f}",
                    [0, 0.5, 0.5], textColorRGB=[0, 0, 0], lifeTime=1)

        for agent in my_world.agent_list:
            if agent.reached_target_position():
                agent.set_next_step()

        p.stepSimulation()

# The following tests all relate to the calculation of the gradient vector
def test_gradient():
    n = 3
    gradient_source = [-2, -2]

    angle_weight = 0
    strain_weight = 0
    gradient_weight = 20
    repulsion_weight = 0

    my_world = World(20, 20, TIME_STEP)

    my_world.set_gradient_source(gradient_source)

    Agent.set_weights([angle_weight, strain_weight, gradient_weight, repulsion_weight])

    # set initial object positions
    initial_robot_positions = [[0, 0, HEIGHT],
                               [0, 1, HEIGHT],
                               [1, 1, HEIGHT]]
    
    # Goal angles for each agent
    goal_angles = [None, 270, None]

    # populates the list of robot objects with robot objects
    for i in range(n):
        my_world.create_agent(initial_robot_positions[i], 0, radius = RADIUS, goal_delta = goal_angles[i])

    # populates the list of tether objects with tether objects
    for i in range(n-1):
        my_world.create_and_anchor_tether(my_world.agent_list[i], my_world.agent_list[i+1], UNSTRETCHED_TETHER_LENGTH, num_segments=10)

    display_axis_labels()

    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)

        for agent in my_world.agent_list:
            agent.sense_gradient(my_world.gradient_source)

        for agent in my_world.agent_list:
            if agent.reached_target_position():
                agent.set_next_step()

        p.stepSimulation()

# Combines the strain and gradient vector functionality
def test_gradient_strain():
    n = 2
    gradient_source = [-2, -2]

    angle_weight = 0
    strain_weight = 50
    gradient_weight = 1
    repulsion_weight = 0

    my_world = World(20, 20, TIME_STEP)

    my_world.set_gradient_source(gradient_source)

    Agent.set_weights([angle_weight, strain_weight, gradient_weight, repulsion_weight])

    # set initial object positions
    initial_robot_positions = [[0, 0, HEIGHT],
                               [0, 1, HEIGHT]]
    
    # Goal angles for each agent
    goal_angles = [None, 270, None]

    # populates the list of robot objects with robot objects
    for i in range(n):
        my_world.create_agent(initial_robot_positions[i], 0, radius = RADIUS, goal_delta = goal_angles[i])

    # populates the list of tether objects with tether objects
    for i in range(n-1):
        my_world.create_and_anchor_tether(my_world.agent_list[i], my_world.agent_list[i+1], UNSTRETCHED_TETHER_LENGTH, num_segments=10)

    display_axis_labels()

    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)

        strain = 0
        for agent in my_world.agent_list:
            if agent.tethers[0]:
                strain = agent.tethers[0].get_strain()

        p.addUserDebugText(f"tether strain = {strain:.2f}",
                    [0, 0.5, 0.5], textColorRGB=[0, 0, 0], lifeTime=1)

        for agent in my_world.agent_list:
            agent.sense_gradient(my_world.gradient_source)

        for agent in my_world.agent_list:
            if agent.reached_target_position():
                agent.set_next_step()

        p.stepSimulation()
        
def test_repulsion_gradient():
    n = 2
    gradient_source = [4, 1]

    angle_weight = 0
    strain_weight = 0
    gradient_weight = 5
    repulsion_weight = 100

    my_world = World(20, 20, TIME_STEP)

    my_world.set_gradient_source(gradient_source)

    Agent.set_weights([angle_weight, strain_weight, gradient_weight, repulsion_weight])

    # set initial object positions
    initial_robot_positions = [[0, 0, HEIGHT],
                               [0, 1, HEIGHT]]

    # populates the list of robot objects with robot objects
    for i in range(n):
        my_world.create_agent(initial_robot_positions[i], 0, radius = RADIUS)

    # populates the list of tether objects with tether objects
    for i in range(n-1):
        my_world.create_and_anchor_tether(my_world.agent_list[i], my_world.agent_list[i+1], UNSTRETCHED_TETHER_LENGTH, num_segments=10)

    # create an obstacle
    my_world.create_obstacle("hexagon", [2, 0])

    display_axis_labels()

    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)

        strain = 0
        for agent in my_world.agent_list:
            if agent.tethers[0]:
                strain = agent.tethers[0].get_strain()

        p.addUserDebugText(f"tether strain = {strain:.2f}",
                    [0, 0.5, 0.5], textColorRGB=[0, 0, 0], lifeTime=1)

        for agent in my_world.agent_list:
            agent.sense_close_range(my_world.obj_list)
            agent.sense_gradient(my_world.gradient_source)

        for agent in my_world.agent_list:
            if agent.reached_target_position():
                agent.set_next_step()

        p.stepSimulation()

# Combines the strain and angle vector functionality
# Successful!
def test_strain_angle():
    n = 3

    my_world = World(20, 20, TIME_STEP)
    my_world.set_gradient_source(GRADIENT_SOURCE)

    Agent.set_weights([2, 20, 0, 0]) # angle, strain, gradient, repulsion

    # set initial object positions
    initial_robot_positions = [[0, 0, HEIGHT],
                               [0, 1, HEIGHT],
                               [1, 1, HEIGHT]]
    
    # Goal angles for each agent
    goal_angles = [None, 90, None]

    # populates the list of robot objects with robot objects
    for i in range(n):
        my_world.create_agent(initial_robot_positions[i], 0, radius = RADIUS, goal_delta = goal_angles[i])

    # populates the list of tether objects with tether objects
    for i in range(n-1):
        my_world.create_and_anchor_tether(my_world.agent_list[i], my_world.agent_list[i+1], UNSTRETCHED_TETHER_LENGTH, num_segments = 5)

    runs = 0

    display_axis_labels()

    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)

        if runs % 200 == 0:
            strain_m = my_world.agent_list[1].tethers[0].get_strain()
            strain_p = my_world.agent_list[1].tethers[1].get_strain()
            p.addUserDebugText(f"tether_m strain = {strain_m:.2f} tether_p strain = {strain_p:.2f}",
                        [0, 0.5, 0.5], textColorRGB=[0, 0, 0], lifeTime=1)
            
        for agent in my_world.agent_list:
            if agent.reached_target_position():
                agent.set_next_step()

        runs = runs + 1

        p.stepSimulation()

def test_repulsion_gradient_strain():
    n = 2
    gradient_source = [4, 1]

    angle_weight = 0
    strain_weight = 20
    gradient_weight = 5
    repulsion_weight = 100

    my_world = World(20, 20, TIME_STEP)

    my_world.set_gradient_source(gradient_source)

    Agent.set_weights([angle_weight, strain_weight, gradient_weight, repulsion_weight])

    # set initial object positions
    initial_robot_positions = [[0, 0, HEIGHT],
                               [0, 1, HEIGHT]]

    # populates the list of robot objects with robot objects
    for i in range(n):
        my_world.create_agent(initial_robot_positions[i], 0, radius = RADIUS)

    # populates the list of tether objects with tether objects
    for i in range(n-1):
        my_world.create_and_anchor_tether(my_world.agent_list[i], my_world.agent_list[i+1], UNSTRETCHED_TETHER_LENGTH, num_segments=10)

    # create an obstacle
    my_world.create_obstacle("hexagon", [2, 0])

    display_axis_labels()

    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)

        strain = 0
        for agent in my_world.agent_list:
            if agent.tethers[0]:
                strain = agent.tethers[0].get_strain()

        p.addUserDebugText(f"tether strain = {strain:.2f}",
                    [0, 0.5, 0.5], textColorRGB=[0, 0, 0], lifeTime=1)

        for agent in my_world.agent_list:
            agent.sense_close_range(my_world.obj_list)
            agent.sense_gradient(my_world.gradient_source)

        for agent in my_world.agent_list:
            if agent.reached_target_position():
                agent.set_next_step()

        p.stepSimulation()

# Combines the gradient and angle vector functionality
def test_gradient_strain_angle():
    n = 3
    gradient_source = [-2, -2]

    my_world = World(20, 20, TIME_STEP)
    my_world.set_gradient_source(gradient_source)

    Agent.set_weights([10, 100, 2, 0]) # angle, strain, gradient, repulsion

    # set initial object positions
    initial_robot_positions = [[0, 0, HEIGHT],
                               [0, 1, HEIGHT],
                               [1, 1, HEIGHT]]
    
    # Goal angles for each agent
    goal_angles = [None, 90, None]

    # populates the list of robot objects with robot objects
    for i in range(n):
        my_world.create_agent(initial_robot_positions[i], 0, radius = RADIUS, goal_delta = goal_angles[i])

    # populates the list of tether objects with tether objects
    for i in range(n-1):
        my_world.create_and_anchor_tether(my_world.agent_list[i], my_world.agent_list[i+1], UNSTRETCHED_TETHER_LENGTH, num_segments = 5)
    
    runs = 0

    display_axis_labels()

    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)

        if runs % 200 == 0:
            strain_m = my_world.agent_list[1].tethers[0].get_strain()
            strain_p = my_world.agent_list[1].tethers[1].get_strain()
            p.addUserDebugText(f"tether_m strain = {strain_m:.2f} tether_p strain = {strain_p:.2f}",
                        [0, 0.5, 0.5], textColorRGB=[0, 0, 0], lifeTime=1)
            
        for agent in my_world.agent_list:
            agent.sense_gradient(my_world.gradient_source)

        for agent in my_world.agent_list:
            if runs%5 == 0:
                agent.set_next_step()

        runs = runs + 1

        p.stepSimulation()

def test_position_movement():
    n = 1

    my_world = World(20, 20, TIME_STEP)

    Agent.set_weights([10, 100, 2, 40]) # angle, strain, gradient, repulsion

    # set initial object positions
    initial_robot_positions = [[0, 0, 0],
                               [0, 1, 0],
                               [1, 1, 0]]

    # populates the list of robot objects with robot objects
    for i in range(n):
        my_world.create_agent(initial_robot_positions[i], 0, radius = RADIUS, mu_static=3)

    # populates the list of tether objects with tether objects
    # for i in range(n-1):
    #     my_world.create_and_anchor_tether(my_world.agent_list[i], my_world.agent_list[i+1], UNSTRETCHED_TETHER_LENGTH, num_segments=5)

    display_axis_labels()

    for agent in my_world.agent_list[::1]:
        agent.next_position = [3, 3]
        agent.move_to()

    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)

        p.stepSimulation()

def test_friction_default():
    n = 3

    my_world = World(20, 20, TIME_STEP)

    Agent.set_weights([10, 100, 2, 40]) # angle, strain, gradient, repulsion

    # set initial object positions
    initial_robot_positions = [[0, 0, 0],
                               [0, 1, 0],
                               [1, 1, 0]]

    # populates the list of robot objects with robot objects
    for i in range(n):
        my_world.create_agent(initial_robot_positions[i], 0, radius = RADIUS, mu_static=3)

    # populates the list of tether objects with tether objects
    for i in range(n-1):
        my_world.create_and_anchor_tether(my_world.agent_list[i], my_world.agent_list[i+1], UNSTRETCHED_TETHER_LENGTH, num_segments=5)

    display_axis_labels()

    for agent in my_world.agent_list[::2]:
        agent.next_position = [5, 5]
        agent.move_to()

    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)

        p.stepSimulation()

def test_friction_high_dynamic_low_static():
    n = 3

    my_world = World(20, 20, TIME_STEP)

    Agent.set_weights([10, 100, 2, 40]) # angle, strain, gradient, repulsion

    # set initial object positions
    initial_robot_positions = [[0, 0, 0],
                               [0, 1, 0],
                               [1, 1, 0]]

    # populates the list of robot objects with robot objects
    for i in range(n):
        my_world.create_agent(initial_robot_positions[i], 0, radius = RADIUS, mu_static=0, mu_dynamic=3)

    # populates the list of tether objects with tether objects
    for i in range(n-1):
        my_world.create_and_anchor_tether(my_world.agent_list[i], my_world.agent_list[i+1], UNSTRETCHED_TETHER_LENGTH, num_segments=5)

    display_axis_labels()

    for agent in my_world.agent_list[1:]:
        agent.next_position = [5, 5]
        agent.move_to()

    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)

        p.stepSimulation()

def test_friction_high_static_low_dynamic():
    n = 3

    my_world = World(20, 20, TIME_STEP)

    Agent.set_weights([10, 100, 2, 40]) # angle, strain, gradient, repulsion

    # set initial object positions
    initial_robot_positions = [[0, 0, 0],
                               [0, 1, 0],
                               [1, 1, 0]]

    # populates the list of robot objects with robot objects
    for i in range(n):
        my_world.create_agent(initial_robot_positions[i], 0, radius = RADIUS, mu_static=3, mu_dynamic=0)

    # populates the list of tether objects with tether objects
    for i in range(n-1):
        my_world.create_and_anchor_tether(my_world.agent_list[i], my_world.agent_list[i+1], UNSTRETCHED_TETHER_LENGTH, num_segments=5)

    display_axis_labels()

    for agent in my_world.agent_list[1:]:
        agent.next_position = [5, 5]
        agent.move_to()

    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)

        p.stepSimulation()

def test_movable_obstacle():
    n = 3
    gradient_source = [6, 0]

    my_world = World(20, 20, TIME_STEP)
    my_world.set_gradient_source(gradient_source)

    Agent.set_weights([10, 100, 2, 40]) # angle, strain, gradient, repulsion

    # set initial object positions
    initial_robot_positions = [[0, 0, HEIGHT],
                               [0, 1, HEIGHT],
                               [1, 1, HEIGHT]]
    
    # Goal angles for each agent
    goal_angles = [None, 90, None]

    # populates the list of robot objects with robot objects
    for i in range(n):
        my_world.create_agent(initial_robot_positions[i], 0, radius = RADIUS, goal_delta = goal_angles[i])

    # populates the list of tether objects with tether objects
    for i in range(n-1):
        my_world.create_and_anchor_tether(my_world.agent_list[i], my_world.agent_list[i+1], UNSTRETCHED_TETHER_LENGTH, num_segments = 5)

    # create an obstacle
    my_world.create_obstacle("hexagon", [2, 0], length=0.5, width=0.5, height=0.5, fixed=False)
    my_world.create_obstacle("triangle", [2, 0], length=0.5, width=0.5, height=0.5, fixed=False)
    my_world.create_obstacle("cube", [2, 0], length=0.5, width=0.5, height=0.5, fixed=False)

    runs = 0

    display_axis_labels()

    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)

        if runs % 200 == 0:
            strain_m = my_world.agent_list[1].tethers[0].get_strain()
            strain_p = my_world.agent_list[1].tethers[1].get_strain()
            p.addUserDebugText(f"tether_m strain = {strain_m:.2f} tether_p strain = {strain_p:.2f}",
                        [0, 0.5, 0.5], textColorRGB=[0, 0, 0], lifeTime=1)
            
        for agent in my_world.agent_list:
            agent.sense_gradient(my_world.gradient_source)
            agent.sense_close_range(my_world.obj_list)

        for agent in my_world.agent_list:
            if runs % 5 == 0:
                agent.set_next_step()

        runs = runs + 1

        p.stepSimulation()
        
def test_all():
    n = 3
    gradient_source = [4, 1]

    my_world = World(20, 20, TIME_STEP)
    my_world.set_gradient_source(gradient_source)

    Agent.set_weights([10, 7, 5, 4]) # angle, strain, gradient, repulsion

    # set initial object positions
    initial_robot_positions = [[0, 0, 0],
                               [0, 1, 0],
                               [1, 1, 0]]
    
    # Goal angles for each agent
    goal_angles = [None, 90, None]

    # populates the list of robot objects with robot objects
    for i in range(n):
        my_world.create_agent(initial_robot_positions[i], 0, radius = RADIUS, goal_delta = goal_angles[i])

    # populates the list of tether objects with tether objects
    for i in range(n-1):
        my_world.create_and_anchor_tether(my_world.agent_list[i], my_world.agent_list[i+1], UNSTRETCHED_TETHER_LENGTH, num_segments = 5)

    # create an obstacle
    my_world.create_obstacle("hexagon", [2, 0])

    runs = 0

    display_axis_labels()

    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)

        strain_m = my_world.agent_list[1].tethers[0].get_strain()
        strain_p = my_world.agent_list[1].tethers[1].get_strain()
        delta = my_world.agent_list[1].get_delta()
        p.addUserDebugText(f"tether_m strain = {strain_m:.2f} tether_p strain = {strain_p:.2f} delta = {delta:.2f}",
                    [0, 0.5, 0.5], textColorRGB=[0, 0, 0], lifeTime=1)

        # if runs % 200 == 0:
        #     strain_m = my_world.agent_list[1].tethers[0].get_strain()
        #     strain_p = my_world.agent_list[1].tethers[1].get_strain()
        #     p.addUserDebugText(f"tether_m strain = {strain_m:.2f} tether_p strain = {strain_p:.2f}",
        #                 [0, 0.5, 0.5], textColorRGB=[0, 0, 0], lifeTime=1)
            
        for agent in my_world.agent_list:
            agent.sense_gradient(my_world.gradient_source)
            agent.sense_close_range(my_world.obj_list)

        for agent in my_world.agent_list:
            if runs % 5 == 0:
                agent.set_next_step()

        runs = runs + 1

        p.stepSimulation()
  

def main():
    test_all()

if __name__ == "__main__":
    main()
