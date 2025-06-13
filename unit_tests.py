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




# The following tests all relate to the calculation of the angle vector

# Successful!
def test_detla_calculation():
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

    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)
        
        if my_world.agent_list[1].reached_target_position():
            my_world.agent_list[1].compute_next_step()
            my_world.agent_list[1].move_to()
        
        p.stepSimulation()     



def main():
    test_angle_vector_90_to_270()

if __name__ == "__main__":
    main()
