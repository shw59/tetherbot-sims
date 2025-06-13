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

def add_axis_labels():
    p.addUserDebugLine([0, 0, 0], [1, 0, 0], lineColorRGB=[0, 0, 1], lineWidth=10, lifeTime=0)
    p.addUserDebugLine([0, 0, 0], [0, 1, 0], lineColorRGB=[1, 0, 0], lineWidth=10, lifeTime=0)
    p.addUserDebugLine([0, 0, 0], [0, 0, 1], lineColorRGB=[.68, .12, .94], lineWidth=10, lifeTime=0)
    p.addUserDebugText("+x", [1, 0, 0], lifeTime=0, textColorRGB=[0, 0, 0])
    p.addUserDebugText("+y", [0, 1, 0], lifeTime=0, textColorRGB=[0, 0, 0])
    p.addUserDebugText("+z", [0, 0, 1], lifeTime=0, textColorRGB=[0, 0, 0])



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
        
    runs = 0

    add_axis_labels()

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
        
    runs = 0

    add_axis_labels()

    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)
        
        if my_world.agent_list[1].reached_target_position():
            my_world.agent_list[1].compute_next_step()
            my_world.agent_list[1].move_to()
        
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
        
    runs = 0

    add_axis_labels()

    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)
        
        if my_world.agent_list[1].reached_target_position():
            my_world.agent_list[1].compute_next_step()
            my_world.agent_list[1].move_to()
        
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
        
    runs = 0

    add_axis_labels()

    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)
        
        if my_world.agent_list[1].reached_target_position():
            my_world.agent_list[1].compute_next_step()
            my_world.agent_list[1].move_to()
        
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
        
    runs = 0

    add_axis_labels()

    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)
        
        if my_world.agent_list[1].reached_target_position():
            my_world.agent_list[1].compute_next_step()
            my_world.agent_list[1].move_to()
        
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
        
    runs = 0

    add_axis_labels()

    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)
        
        if my_world.agent_list[1].reached_target_position():
            my_world.agent_list[1].compute_next_step()
            my_world.agent_list[1].move_to()
        
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
        
    runs = 0

    add_axis_labels()

    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)
        
        if my_world.agent_list[1].reached_target_position():
            my_world.agent_list[1].compute_next_step()
            my_world.agent_list[1].move_to()
        
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
        
    runs = 0

    add_axis_labels()

    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)
        
        if my_world.agent_list[1].reached_target_position():
            my_world.agent_list[1].compute_next_step()
            my_world.agent_list[1].move_to()
        
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
        
    runs = 0

    add_axis_labels()

    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)
        
        if my_world.agent_list[1].reached_target_position():
            my_world.agent_list[1].compute_next_step()
            my_world.agent_list[1].move_to()
        
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
        
    runs = 0

    add_axis_labels()

    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)
        
        if my_world.agent_list[1].reached_target_position():
            my_world.agent_list[1].compute_next_step()
            my_world.agent_list[1].move_to()
        
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

    add_axis_labels()

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
                agent.compute_next_step()
                agent.move_to()

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

    add_axis_labels()

    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)

        for agent in my_world.agent_list:
            agent.sense_gradient(my_world.gradient_source)

        for agent in my_world.agent_list:
            if agent.reached_target_position():
                agent.compute_next_step()
                agent.move_to()

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

    add_axis_labels()

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
                agent.compute_next_step()
                agent.move_to()

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

    add_axis_labels()

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
                agent.compute_next_step()
                agent.move_to()

        p.stepSimulation()

# Combines the strain and angle vector functionality
# Successful!
def unit_test_strain_angle():
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
    add_axis_labels()
    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)
        if runs%200 == 0:
            strain_m = my_world.agent_list[1].tethers[0].get_strain()
            strain_p = my_world.agent_list[1].tethers[1].get_strain()
            p.addUserDebugText(f"tether_m strain = {strain_m:.2f} tether_p strain = {strain_p:.2f}",
                        [0, 0.5, 0.5], textColorRGB=[0, 0, 0], lifeTime=1)
        for agent in my_world.agent_list:
            if agent.reached_target_position():
                agent.compute_next_step()
                agent.move_to()
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

    add_axis_labels()

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
                agent.compute_next_step()
                agent.move_to()

        p.stepSimulation()


# Combines the gradient and angle vector functionality
def unit_test_gradient_strain_angle():
    n = 3
    gradient_source = [-2, -2]
    my_world = World(20, 20, TIME_STEP)
    my_world.set_gradient_source(GRADIENT_SOURCE)
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
    add_axis_labels()
    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)
        if runs%200 == 0:
            strain_m = my_world.agent_list[1].tethers[0].get_strain()
            strain_p = my_world.agent_list[1].tethers[1].get_strain()
            p.addUserDebugText(f"tether_m strain = {strain_m:.2f} tether_p strain = {strain_p:.2f}",
                        [0, 0.5, 0.5], textColorRGB=[0, 0, 0], lifeTime=1)
        for agent in my_world.agent_list:
            agent.sense_gradient(my_world.gradient_source)
        for agent in my_world.agent_list:
            if runs%5 == 0:
                agent.compute_next_step()
                agent.move_to()
            if agent.reached_target_position():
                agent.compute_next_step()
                agent.move_to()
        runs = runs + 1
        p.stepSimulation()
        
  
def main():
    unit_test_gradient_strain_angle()

if __name__ == "__main__":
    main()
