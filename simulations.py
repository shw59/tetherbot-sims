"""
simulations.py

This file contains all experimental simulation functions that we are collecting data from.
"""

import pybullet as p
from world import World
from agent import Agent
import utils
import simulation_utils as sims_utils
import math
import numpy as np
import random
import time

class Simulation:
    run_debounce = 35 # number of iterations to wait before checking tether slackness
    debounce_threshold = 5 # number of iterations that the tether can be slack before simulation is considered failed

    def __init__(self, time_step, agent_mass, agent_radius, agent_height, agent_max_speed, agent_drive_power, agent_static_mu, agent_dynamic_mu, 
                 unstretched_tether_length, tether_youngs_modulus, tether_diameter, weight_a, weight_s, weight_g, weight_r, sensing_period, logging_period, gui_on):
        """
        Initializes a simulation object with the given attributes that apply to all simulations run from that instance.
        """
        self.time_step = time_step
        self.sensing_period = sensing_period
        self.logging_period = logging_period
        self.gui_on = gui_on

        self.agent_mass = agent_mass
        self.agent_radius = agent_radius
        self.agent_height = agent_height
        self.agent_max_speed = agent_max_speed
        self.agent_drive_power = agent_drive_power
        self.agent_static_mu = agent_static_mu
        self.agent_dynamic_mu = agent_dynamic_mu

        self.unstretched_tether_length = unstretched_tether_length
        self.tether_youngs_modulus = tether_youngs_modulus
        self.tether_diameter = tether_diameter

        self.weight_angle = weight_a
        self.weight_strain = weight_s
        self.weight_gradient = weight_g
        self.weight_repulsion = weight_r

        self.sim_failed = False
        self.debounce_count = 0
        self.old_debounce_count = 0

    def building_plan(self):
        """
        Generates an environment that resembles a building plan. The idea is to create a 2D map for a group
        of agents to traverse through with both movable and non-movable objects.
        """
        self.reset_simulation()
        
        n = 5

        gradient = [30, 40]

        my_world = World(120, 120, self.time_step, self.gui_on)

        my_world.set_gradient_source(gradient)

        Agent.set_weights([self.weight_angle, self.weight_strain, self.weight_gradient, self.weight_repulsion])

        # angles = [None, 180, 180, 180, 180, 180, None]
        angles = [None, 180, 180, 180, None]
        
        initial_robot_positions = sims_utils.basic_starting_positions(self.unstretched_tether_length, n, angles, [-14,-20,0], "+x")
        
        goal_angles = [None, 250, 250, 250, None]

        # populates the list of robot objects with robot objects
        for i in range(n):
            my_world.create_agent(initial_robot_positions[i], 0, radius = self.agent_radius, goal_delta = goal_angles[i], 
                                  mass=self.agent_mass, height=self.agent_height, color=(0.5, 0.5, 1, 1), mu_static=self.agent_static_mu,
                                  mu_dynamic=self.agent_dynamic_mu, max_velocity=self.agent_max_speed, drive_power=self.agent_drive_power)

        # populates the list of tether objects with tether objects
        for i in range(n-1):
            my_world.create_and_anchor_tether(my_world.agent_list[i], my_world.agent_list[i+1], self.unstretched_tether_length, self.tether_youngs_modulus, self.tether_diameter, num_segments = 10)


        # bottom room, bottom right wall
        my_world.create_obstacle("cube", [-6, -18], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-5, -18], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-4, -18], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-3, -18], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-2, -18], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-1, -18], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [0, -18], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [1, -18], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [2, -18], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [3, -18], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [4, -18], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [5, -18], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [6, -18], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [7, -18], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)

        # bottom room, right wall
        my_world.create_obstacle("cube", [7, -17], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [7, -16], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [7, -15], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [7, -14], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [7, -13], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [7, -12], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [7, -11], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [7, -10], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [7, -9], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [7, -8], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [7, -7], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [7, -6], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [7, -5], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [7, -4], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [7, -3], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)

        # bottom room, bottom left wall
        my_world.create_obstacle("cube", [-13, -18], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-14, -18], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-15, -18], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-16, -18], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-17, -18], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-18, -18], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-19, -18], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-20, -18], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)

        # bottom room, left wall
        my_world.create_obstacle("cube", [-15, -17], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-15, -16], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-15, -15], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-15, -14], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-15, -13], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-15, -12], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-15, -11], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-15, -10], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-15, -9], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-15, -8], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-15, -7], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-15, -6], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-15, -5], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-15, -4], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-15, -3], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)

        # bottom room, top wall
        my_world.create_obstacle("cube", [-14, -3], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-13, -3], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-12, -3], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-11, -3], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-10, -3], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-9, -3], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-8, -3], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-7, -3], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-6, -3], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-5, -3], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-4, -3], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-3, -3], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-2, -3], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-1, -3], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [0, -3], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [1, -3], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [2, -3], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [3, -3], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [4, -3], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [8, -3], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [9, -3], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [10, -3], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [11, -3], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)

        # hallway
        my_world.create_obstacle("cube", [4, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [4, -1], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [4, 0], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [5, 0], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [6, 0], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [7, 0], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [8, 0], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [9, 0], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [10, 0], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [11, 0], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [12, 0], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [13, 0], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [14, 0], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [15, 0], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [16, 0], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        

        # top room, bottom wall
        my_world.create_obstacle("cube", [12, -3], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [13, -3], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [14, -3], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [15, -3], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [16, -3], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [17, -3], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [18, -3], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [19, -3], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [20, -3], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [21, -3], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [22, -3], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [23, -3], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [24, -3], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [25, -3], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [26, -3], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [27, -3], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [28, -3], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)

        # top room, right wall
        my_world.create_obstacle("cube", [28, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [28, -1], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [28, 0], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [28, 1], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [28, 2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [28, 3], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [28, 4], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [28, 5], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [28, 6], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [28, 7], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [28, 8], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [28, 9], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [28, 10], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [28, 11], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [28, 12], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [28, 13], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [28, 14], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)

        # top room, left wall
        my_world.create_obstacle("cube", [16, 1], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [16, 2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [16, 3], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [16, 4], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [16, 5], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [16, 6], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [16, 7], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [16, 8], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [16, 9], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [16, 10], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [16, 11], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [16, 12], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [16, 13], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [16, 14], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)

        # top room, top left
        my_world.create_obstacle("cube", [16, 15], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [17, 15], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)

        # top room, top right
        my_world.create_obstacle("cube", [28, 15], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [27, 15], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)

        # big couch
        my_world.create_obstacle("cube", [-10, -15], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-10, -14], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-10, -13], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-10, -12], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-10, -11], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-10, -10], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)

        my_world.create_obstacle("cube", [-9, -15], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-9, -14], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-9, -13], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-9, -12], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-9, -11], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-9, -10], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)

        my_world.create_obstacle("cube", [-8, -10], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-8, -11], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-7, -10], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-7, -11], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-6, -10], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-6, -11], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-5, -10], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-5, -11], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-4, -10], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-4, -11], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-3, -10], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-3, -11], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-2, -10], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        my_world.create_obstacle("cube", [-2, -11], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)

        # square table
        # my_world.create_obstacle("cube", [-1, -10], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        # my_world.create_obstacle("cube", [0, -10], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        # my_world.create_obstacle("cube", [1, -10], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        # my_world.create_obstacle("cube", [2, -10], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        # my_world.create_obstacle("cube", [-1, -9], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        # my_world.create_obstacle("cube", [0, -9], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        # my_world.create_obstacle("cube", [1, -9], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        # my_world.create_obstacle("cube", [2, -9], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        # my_world.create_obstacle("cube", [-1, -8], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        # my_world.create_obstacle("cube", [0, -8], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        # my_world.create_obstacle("cube", [1, -8], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        # my_world.create_obstacle("cube", [2, -8], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)

        # block representing position of gradient
        my_world.create_obstacle("cube", gradient, length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)

        # things to collect
        sims_utils.generate_obstacles(my_world, [16, 0], [25, 14], 5, "cylinder", 0.5, 0.5, False)
        


        runs = 0

        agent_to_update_next = 0

        shuffled_list = random.sample(my_world.agent_list, k=len(my_world.agent_list))

        # main simulation loop
        while my_world.id.isConnected() and math.dist(my_world.agent_list[0].get_pose()[0], gradient) > 10:
            my_world.id.getCameraImage(320,200)

            

            self.debounce_count = 0

            for agent in shuffled_list:
                # if agent.tethers[0] is not None:
                    # print(agent.tethers[0].get_strain())
                # else:
                    # print(agent.tethers[1].get_strain())




                if runs > Simulation.run_debounce and agent.is_tether_slack():
                    self.debounce_count += 1
                # else:
                #     self.debounce_count = 0

                if runs%Simulation.run_debounce == 0:

                    if self.old_debounce_count >= Simulation.debounce_threshold and self.debounce_count >= Simulation.debounce_threshold:
                        self.sim_failed = True
                        break

                    self.old_debounce_count = self.debounce_count

                # if self.debounce_count >= Simulation.debounce_threshold:
                #     self.sim_failed = True
                #     break

                agent.sense_gradient(my_world.gradient_source)
                agent.sense_close_range(my_world.obj_list, sensing_mode=2)

            if runs % self.sensing_period == 0:
                for i in range(len(shuffled_list)):
                    if i == agent_to_update_next:
                        shuffled_list[i].set_next_step()
                    
                agent_to_update_next = agent_to_update_next + 1

                if agent_to_update_next >= len(shuffled_list):
                    agent_to_update_next = 0

            if runs % 1000 == 0:
                sims_utils.screenshot_gui(ss_filename=f"data/figures/time_step_{runs}_building_plan_screenshot.png")

            if self.sim_failed:
                break

            runs = runs + 1
            
            my_world.id.stepSimulation()

        my_world.id.disconnect()

        return None
    

    def storm_drain(self):
        """
        Generates an environment like the one described in the research paper. The robots go through a pipe, get
        to a storm drain, and then collect debris in the storm drain and bring it to the top of the tank.
        """
        self.reset_simulation()
        
        n = 7

        gradient = [7.5, 50]

        my_world = World(120, 120, self.time_step, self.gui_on)

        my_world.set_gradient_source(gradient)

        Agent.set_weights([self.weight_angle, self.weight_strain, self.weight_gradient, self.weight_repulsion])

        angles = [None, 180, 180, 180, 180, 180, None]
        
        initial_robot_positions = sims_utils.basic_starting_positions(self.unstretched_tether_length, n, angles, [-30,-7,0], "+x")
        
        goal_angles = [None, 180, 270, 180, 270, 180, None]

        # populates the list of robot objects with robot objects
        for i in range(n):
            my_world.create_agent(initial_robot_positions[i], 0, radius = self.agent_radius, goal_delta = goal_angles[i], 
                                  mass=self.agent_mass, height=self.agent_height, color=(0.5, 0.5, 1, 1), mu_static=self.agent_static_mu,
                                  mu_dynamic=self.agent_dynamic_mu, max_velocity=self.agent_max_speed, drive_power=self.agent_drive_power)

        # populates the list of tether objects with tether objects
        for i in range(n-1):
            my_world.create_and_anchor_tether(my_world.agent_list[i], my_world.agent_list[i+1], self.unstretched_tether_length, self.tether_youngs_modulus, self.tether_diameter, num_segments = 10)

        # sides of sewer tank

        for i in range(30):
            my_world.create_obstacle("cube", [-10, i], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
            my_world.create_obstacle("cube", [25, i], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)

        # top of the sewer tank
        my_world.create_obstacle("cube", [-10, 30], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [-9, 30], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [-8, 30], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [-7, 30], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [-6, 30], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [-5, 30], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [-4, 30], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [-3, 30], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [-2, 30], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [-1, 30], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [0, 30], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [1, 30], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [2, 30], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [3, 30], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [4, 30], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [11, 30], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [12, 30], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [13, 30], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [14, 30], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [15, 30], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [16, 30], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [17, 30], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [18, 30], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [19, 30], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [20, 30], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [21, 30], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [22, 30], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [23, 30], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [24, 30], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [25, 30], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)


        # bottom of the sewer tank
        my_world.create_obstacle("cube", [-9, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [-8, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [-7, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [-6, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [-5, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [-4, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [-3, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [-2, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [-1, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [0, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [1, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [2, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [3, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [4, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [5, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [6, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [7, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [8, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [9, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [10, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [11, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [12, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [13, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [14, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [15, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [15, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [16, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [17, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [18, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [19, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [20, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [21, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [22, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [23, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [24, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [25, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [25, -1], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)

        # tube connected to the tank
        my_world.create_obstacle("cube", [-10, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [-11, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [-12, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [-13, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [-14, -2], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [-10, 0], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [-11, 0], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [-12, 0], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [-13, 0], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [-14, 0], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [-15, 0], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)


        # diagonal part
        # my_world.create_obstacle("hexagon", [-14.25, -2.25], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        # my_world.create_obstacle("hexagon", [-14.5, -2.5], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-14.75, -2], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-15, -2.25], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-15.25, -2.5], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-15.5, -2.75], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-15.75, -3], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-16, -3.25], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-16.25, -3.5], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-16.5, -3.75], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-16.75, -4], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-17, -4.25], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-17.25, -4.5], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-17.5, -4.75], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-17.75, -5], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-18, -5.25], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-18.25, -5.5], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-18.5, -5.75], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-18.75, -6], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-19, -6.25], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-19.25, -6.5], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-19.5, -6.75], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-19.75, -7], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-20, -7.25], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-20.25, -7.50], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-20.5, -7.75], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-20.75, -8], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-21, -8], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-21.25, -8], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-15.75, -.5], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-15.75, -.75], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-16, -1], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-16.25, -1.25], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-16.5, -1.5], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-16.75, -1.75], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-17, -2], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-17.25, -2.25], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-17.5, -2.5], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-17.75, -2.75], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-18, -3], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-18.25, -3.25], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-18.5, -3.5], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-18.75, -3.75], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-19, -4], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-19.25, -4.25], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-19.5, -4.5], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-19.75, -4.75], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-20, -5], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-20.25, -5.25], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-20.5, -5.5], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-20.75, -5.75], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-21, -6], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("hexagon", [-21.25, -6], length=0.35, width=0.35, color=(0, 0, 0, 1), fixed=True, height=0.5)

        # tube where the agents start
        my_world.create_obstacle("cube", [-22, -6], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [-23, -6], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [-24, -6], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [-25, -6], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [-26, -6], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [-27, -6], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [-28, -6], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [-29, -6], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [-30, -6], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [-22, -8], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [-23, -8], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [-24, -8], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [-25, -8], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [-26, -8], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [-27, -8], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [-28, -8], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [-29, -8], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        my_world.create_obstacle("cube", [-30, -8], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)

        sims_utils.generate_obstacles(my_world, [25, 30], [-10, -2], 100, "cylinder", 0.5, 0.5, False)
        
        runs = 0

        agent_to_update_next = 0

        shuffled_list = random.sample(my_world.agent_list, k=len(my_world.agent_list))

        # main simulation loop
        while my_world.id.isConnected() and math.dist(my_world.agent_list[0].get_pose()[0], gradient) > 10:
            my_world.id.getCameraImage(320,200)

            

            self.debounce_count = 0

            for agent in shuffled_list:
                # if agent.tethers[0] is not None:
                    # print(agent.tethers[0].get_strain())
                # else:
                    # print(agent.tethers[1].get_strain())




                if runs > Simulation.run_debounce and agent.is_tether_slack():
                    self.debounce_count += 1
                # else:
                #     self.debounce_count = 0

                if runs%Simulation.run_debounce == 0:

                    if self.old_debounce_count >= Simulation.debounce_threshold and self.debounce_count >= Simulation.debounce_threshold:
                        self.sim_failed = True
                        break

                    self.old_debounce_count = self.debounce_count

                # if self.debounce_count >= Simulation.debounce_threshold:
                #     self.sim_failed = True
                #     break

                agent.sense_gradient(my_world.gradient_source)
                agent.sense_close_range(my_world.obj_list, sensing_mode=2)

            if runs % self.sensing_period == 0:
                for i in range(len(shuffled_list)):
                    if i == agent_to_update_next:
                        shuffled_list[i].set_next_step()
                    
                agent_to_update_next = agent_to_update_next + 1

                if agent_to_update_next >= len(shuffled_list):
                    agent_to_update_next = 0

            if runs % 1000 == 0:
                sims_utils.screenshot_gui(ss_filename=f"data/figures/time_step_{runs}_storm_drain_screenshot.png")

            if self.sim_failed:
                break

            runs = runs + 1
            
            my_world.id.stepSimulation()

        my_world.id.disconnect()

        return None

    def obstacle_avoidance(self, n, y_offset, angle_off_y, a_weight = 10, s_weight = 15, g_weight = 10, r_weight = 3, gradient = [100,0], obst_pos = [6,0], obst_radius = 1, obst_height = 1, obst_type = "hexagon", stop=2000, trial = 0):
        """
        Generates a very simple formation of agents in order to test the hysteresis.
        """
        self.reset_simulation()

        my_world = World(200, 200, self.time_step, gui_on=self.gui_on)

        my_world.set_gradient_source(gradient)

        Agent.set_weights([a_weight, s_weight, g_weight, r_weight])
        
        initial_agent_positions = sims_utils.angle_and_position_offset(n, angle_off_y, y_offset, self.unstretched_tether_length)

        goals = []
        for i in range(n):
            if i == 0 or i == (n-1):
                goals.append(None)
            else:
                goals.append(180)

        # populates the list of robot objects with agent objects
        for i in range(n):
            my_world.create_agent(initial_agent_positions[i], 0, radius = self.agent_radius, goal_delta = goals[i], 
                                  mass=self.agent_mass, height=self.agent_height, color=(0.5, 0.5, 1, 1), mu_static=self.agent_static_mu,
                                  mu_dynamic=self.agent_dynamic_mu, max_velocity=self.agent_max_speed, drive_power=self.agent_drive_power)

        # populates the list of tether objects with tether objects
        for i in range(n-1):
            my_world.create_and_anchor_tether(my_world.agent_list[i], my_world.agent_list[i+1], self.unstretched_tether_length, self.tether_youngs_modulus, self.tether_diameter, num_segments = 10)

        my_world.create_obstacle(obst_type, obst_pos, length=obst_radius, width=obst_radius, color=(1, 0, 1, 1), fixed=True, height=obst_height)

        my_world.display_axis_labels()
        
        runs = 0

        agent_to_update_next = 0

        shuffled_list = random.sample(my_world.agent_list, k=len(my_world.agent_list))

        # log_file = str(angle_off_y) + "_degree_" + str(y_offset) + "_offset_.csv"
        log_file = "data/trial" + str(trial) + "_degree" + str(angle_off_y) + "_offset" + str(y_offset) + ".csv"

        log_header = ['Timestep']

        for i in range(n):
            log_header.append('agent_' + str(i) + '_x')
            log_header.append('agent_' + str(i) + '_y')
            log_header.append('agent_' + str(i) + '_velocity')
        
        # main simulation loop
        while (runs <= stop) and (my_world.id.isConnected()):

            self.debounce_count = 0

            if runs%30:
                my_world.id.getCameraImage(320,200)

            for agent in shuffled_list:
                if runs > Simulation.run_debounce and agent.is_tether_slack():
                    self.debounce_count += 1

                if runs%Simulation.run_debounce == 0:

                    if self.old_debounce_count >= Simulation.debounce_threshold and self.debounce_count >= Simulation.debounce_threshold:
                        self.sim_failed = True
                        break

                    self.old_debounce_count = self.debounce_count

                agent.sense_gradient(my_world.gradient_source)
                agent.sense_close_range(my_world.obj_list, sensing_mode=2)

            if runs % self.sensing_period == 0:
                for i in range(len(shuffled_list)):
                    if i == agent_to_update_next:
                        shuffled_list[i].set_next_step()
                
                agent_to_update_next = agent_to_update_next + 1

                if agent_to_update_next >= len(shuffled_list):
                    agent_to_update_next = 0

            if runs % self.logging_period == 0:
                data = [runs]
                for agent in my_world.agent_list:
                    data.append(round(agent.get_pose()[0][0], 5))
                    data.append(round(agent.get_pose()[0][1], 5))
                    data.append(agent.get_velocity())
                # data.append(True)
            
                sims_utils.log_to_csv(log_file, data, header=log_header)

            if self.sim_failed:
                break

            runs = runs + 1
            
            my_world.id.stepSimulation()

        my_world.id.disconnect()

        return log_file, self.sim_failed


    def one_agent_follows_gradient(self):
        """
        """
        self.reset_simulation()
        
        n = 5

        gradient = [20, 0]

        my_world = World(120, 120, self.time_step, self.gui_on)

        my_world.set_gradient_source(gradient)

        Agent.set_weights([self.weight_angle, self.weight_strain, self.weight_gradient, self.weight_repulsion])

        angles = [None, 180, 180, 180, None]
        
        initial_robot_positions = sims_utils.basic_starting_positions(self.unstretched_tether_length, n, angles, [0,0,0], "+y")
        
        goal_angles = [None, 180, 180, 180, None]

        # populates the list of robot objects with robot objects
        for i in range(n):
            if i == 0:
                my_world.create_agent(initial_robot_positions[i], 0, radius = self.agent_radius, goal_delta = goal_angles[i], 
                                  mass=self.agent_mass, height=self.agent_height, color=(0.5, 0.5, 1, 1), mu_static=self.agent_static_mu,
                                  mu_dynamic=self.agent_dynamic_mu, max_velocity=self.agent_max_speed, drive_power=self.agent_drive_power, only_gradient=True, dont_care_about_gradient = False)
            else:
                my_world.create_agent(initial_robot_positions[i], 0, radius = self.agent_radius, goal_delta = goal_angles[i], 
                                  mass=self.agent_mass, height=self.agent_height, color=(0.5, 0.5, 1, 1), mu_static=self.agent_static_mu,
                                  mu_dynamic=self.agent_dynamic_mu, max_velocity=self.agent_max_speed, drive_power=self.agent_drive_power, only_gradient=False, dont_care_about_gradient = True)
            

        # populates the list of tether objects with tether objects
        for i in range(n-1):
            my_world.create_and_anchor_tether(my_world.agent_list[i], my_world.agent_list[i+1], self.unstretched_tether_length, self.tether_youngs_modulus, self.tether_diameter, num_segments = 10)
        
        runs = 0

        agent_to_update_next = n - 1

        update_cycles_to_weight = 3
        
        update_cycles_weighted = 0

        # shuffled_list = random.sample(my_world.agent_list, k=len(my_world.agent_list))

        time.sleep(5)

        # main simulation loop
        while my_world.id.isConnected() and math.dist(my_world.agent_list[0].get_pose()[0], gradient) > 10:
            my_world.id.getCameraImage(320,200)

            self.debounce_count = 0

            for agent in my_world.agent_list:
                # if agent.tethers[0] is not None:
                    # print(agent.tethers[0].get_strain())
                # else:
                    # print(agent.tethers[1].get_strain())

                if runs > Simulation.run_debounce and agent.is_tether_slack():
                    self.debounce_count += 1
                # else:
                #     self.debounce_count = 0

                if runs%Simulation.run_debounce == 0:

                    if self.old_debounce_count >= Simulation.debounce_threshold and self.debounce_count >= Simulation.debounce_threshold:
                        self.sim_failed = True
                        break

                    self.old_debounce_count = self.debounce_count

                # if self.debounce_count >= Simulation.debounce_threshold:
                #     self.sim_failed = True
                #     break

                agent.sense_gradient(my_world.gradient_source)
                agent.sense_close_range(my_world.obj_list, sensing_mode=2)

            if runs % self.sensing_period == 0:
                for i in range(len(my_world.agent_list) - 1 , -1, -1):
                    if i == 0 and update_cycles_weighted < update_cycles_to_weight:
                        update_cycles_weighted = update_cycles_weighted + 1
                    elif i == 0 and update_cycles_weighted >= update_cycles_to_weight:
                        my_world.agent_list[i].set_next_step()
                        update_cycles_weighted = 0
                    else:
                        if i == agent_to_update_next:
                            my_world.agent_list[i].set_next_step()
                    
                # if update_cycles_weighted > update_cycles_to_weight:
                #     update_cycles_weighted = 0

                agent_to_update_next = agent_to_update_next - 1

                if agent_to_update_next < 0:
                    agent_to_update_next = len(my_world.agent_list) - 1

            if runs % 200 == 0:
                sims_utils.screenshot_gui(ss_filename=f"data/figures/time_step_{runs}_one_agent_follows_g_screenshot.png")

            if self.sim_failed:
                break

            runs = runs + 1
            
            my_world.id.stepSimulation()

        my_world.id.disconnect()

        return None



    def tow_failed_agents_trial(self, n, trial_num, time_steps, failed_agent_num):
        """
        This experiment takes a group of n agents in a W formation and causes one of them to fail at t = 100.
        The other agents attempt to tow the failed agent as the collective advances towards the gradient source goal.
        """
        self.reset_simulation()

        my_world = World(150, 20, self.time_step, self.gui_on)

        gradient_source = [100, 0]

        my_world.set_gradient_source(gradient_source)

        Agent.set_weights([self.weight_angle, self.weight_strain, self.weight_gradient, self.weight_repulsion])

        start_angles = [225]
        for i in range(1, n - 1):
            if i % 2 == 0:
                start_angles.append(270)
            else:
                start_angles.append(90)
        start_angles.append(None)
        
        initial_agent_positions = sims_utils.basic_starting_positions(self.unstretched_tether_length, n, start_angles, [-2, (n - 1) * self.unstretched_tether_length / 2, 0], "+y")

        goal_angles = [None] + start_angles[1:]

        # populates the list of robot objects with agent objects
        for i in range(n):
            my_world.create_agent(initial_agent_positions[i], 0, radius = self.agent_radius, goal_delta = goal_angles[i], 
                            mass=self.agent_mass, height=self.agent_height, color=(0.5, 0.5, 1, 1), mu_static=self.agent_static_mu,
                            mu_dynamic=self.agent_dynamic_mu, max_velocity=self.agent_max_speed, drive_power=self.agent_drive_power)

        # populates the list of tether objects with tether objects
        for i in range(n-1):
            my_world.create_and_anchor_tether(my_world.agent_list[i], my_world.agent_list[i+1], self.unstretched_tether_length, self.tether_youngs_modulus, self.tether_diameter, num_segments=10)

        my_world.display_axis_labels()
        
        log_file = f"data/tow_failed_agents_trial{trial_num}_agent{failed_agent_num}_failed.csv"
        runs = 0

        agent_to_update_next = 0

        shuffled_list = random.sample(my_world.agent_list, k=len(my_world.agent_list))

        # main simulation loop
        while my_world.id.isConnected() and runs <= time_steps:
            my_world.id.getCameraImage(320,200)

            self.debounce_count = 0

            if runs % self.logging_period == 0:
                failed_x = 0
                non_failed_x = []
                for i in range(len(my_world.agent_list)):
                    if i != failed_agent_num:
                        non_failed_x.append(my_world.agent_list[i].get_pose()[0][0])
                    else:
                        failed_x = my_world.agent_list[i].get_pose()[0][0]

                sum_velocities = 0
                for agent in my_world.agent_list:
                    sum_velocities += agent.get_velocity()
                avg_velocity = sum_velocities / n
                    
                csv_row = [runs, failed_x, avg_velocity] + non_failed_x
                sims_utils.log_to_csv(log_file, csv_row, header=["time step", "failed agent x-position", "formation velocity", "other agent x-positions"] + ["" for _ in range(n - 2)])

            if runs == 500:
                my_world.agent_list[failed_agent_num].failed = True

            for agent in shuffled_list:
                # if agent.tethers[0] is not None:
                #     print(agent.tethers[0].get_strain())
                # else:
                #     print(agent.tethers[1].get_strain())

                if runs > Simulation.run_debounce and agent.is_tether_slack():
                    self.debounce_count += 1

                if runs%Simulation.run_debounce == 0:

                    if self.old_debounce_count >= Simulation.debounce_threshold and self.debounce_count >= Simulation.debounce_threshold:
                        self.sim_failed = True
                        break

                    self.old_debounce_count = self.debounce_count

                agent.sense_gradient(my_world.gradient_source)
                agent.sense_close_range(my_world.obj_list, sensing_mode=2)

            if self.sim_failed:
                break

            if runs % self.sensing_period == 0:
                for i in range(len(shuffled_list)):
                    if i == agent_to_update_next:
                        shuffled_list[i].set_next_step()

                agent_to_update_next = agent_to_update_next + 1

                if agent_to_update_next >= len(shuffled_list):
                    agent_to_update_next = 0

            runs = runs + 1
            
            my_world.id.stepSimulation()
        
        my_world.id.disconnect()

        return log_file, self.sim_failed

    def object_capture_trial(self, n, trial_num, time_steps, num_objects, offset, maintain_line):
        """
        This experiment takes a group of n agents and places them in a line. The agents then attempt to collect randomly placed movable obstacles.
        The collective may either attempt to maintain a straight line or have no goal angle.
        """
        self.reset_simulation()

        my_world = World(150, 150, self.time_step, self.gui_on)

        gradient_source = [100, 0]

        my_world.set_gradient_source(gradient_source)

        Agent.set_weights([self.weight_angle, self.weight_strain, self.weight_gradient, self.weight_repulsion])

        start_angles = [None] + [180] * (n - 2) + [None]
        
        initial_agent_positions = sims_utils.basic_starting_positions(self.unstretched_tether_length, n, start_angles, [-2, -(n - 1) * self.unstretched_tether_length / 2 + offset, 0], "+y")

        if maintain_line:
            goal_angles = start_angles[:]
        else:
            goal_angles = [None] * n

        # populates the list of robot objects with agent objects
        for i in range(n):
            my_world.create_agent(initial_agent_positions[i], 0, radius = self.agent_radius, goal_delta = goal_angles[i], 
                                  mass=self.agent_mass, height=self.agent_height, color=(0.5, 0.5, 1, 1), mu_static=self.agent_static_mu,
                                  mu_dynamic=self.agent_dynamic_mu, max_velocity=self.agent_max_speed, drive_power=self.agent_drive_power)

        # populates the list of tether objects with tether objects
        for i in range(n-1):
            my_world.create_and_anchor_tether(my_world.agent_list[i], my_world.agent_list[i+1], self.unstretched_tether_length, self.tether_youngs_modulus, self.tether_diameter, num_segments=10)

        sims_utils.generate_obstacles(my_world, [0, -5], [5, 5], num_objects, "cylinder", 0.1, 0.1, False)

        my_world.display_axis_labels()
        
        log_file = f"data/object_capture_maintain_line_{maintain_line}_trial{trial_num}_objects{num_objects}_offset{offset}.csv"
        runs = 0
        agent_to_update_next = 0
        shuffled_list = random.sample(my_world.agent_list, k=len(my_world.agent_list))

        obj_collected = 0

        collective_radius_avgs = []

        # main simulation loop
        while my_world.id.isConnected() and runs <= time_steps:

            self.debounce_count = 0

            my_world.id.getCameraImage(320,200)

            if runs % self.logging_period == 0:
                obstacle_pos_flattened = []
                for obj in my_world.obj_list:
                    if obj.label == "obstacle":
                        obstacle_pos_flattened.append(obj.get_pose()[0][0])
                        obstacle_pos_flattened.append(obj.get_pose()[0][1])
                        for agent in my_world.agent_list:
                            if math.dist(agent.get_pose()[0], obj.get_pose()[0]) <= 2:
                                    if not obj.collected:
                                        obj_collected += 1
                                        obj.collected = True
                                        break
                            else:
                                if obj.collected:
                                    obj_collected -= 1
                                    obj.collected = False
                
                agent_pos = [agent.get_pose()[0] for agent in my_world.agent_list]

                # sliding window average
                collective_radius_avgs.append(utils.get_collective_radius(agent_pos))
                
                if len(collective_radius_avgs) > 5:
                    collective_radius_avgs.pop(0)

                collective_radius = np.mean(collective_radius_avgs)

                agent_pos_flattened = []
                for pos in agent_pos:
                    agent_pos_flattened.append(pos[0])
                    agent_pos_flattened.append(pos[1])
                csv_row = [runs, collective_radius, obj_collected] + agent_pos_flattened + obstacle_pos_flattened
                sims_utils.log_to_csv(log_file, csv_row, header=["time step", "collective radius", "# of objects collected", "agent positions"] + ["" for _ in range(n * 2 - 1)] + ["obstacle positions"] + ["" for _ in range(num_objects * 2 - 1)])

            for agent in shuffled_list:
                if runs > Simulation.run_debounce and agent.is_tether_slack():
                    self.debounce_count += 1

                if runs%Simulation.run_debounce == 0:

                    if self.old_debounce_count >= Simulation.debounce_threshold and self.debounce_count >= Simulation.debounce_threshold:
                        self.sim_failed = True
                        break

                    self.old_debounce_count = self.debounce_count
                    
                agent.sense_gradient(my_world.gradient_source)
                agent.sense_close_range(my_world.obj_list, sensing_mode=2)

            if self.sim_failed:
                break

            if runs % self.sensing_period == 0:
                for i in range(len(shuffled_list)):
                    if i == agent_to_update_next:
                        shuffled_list[i].set_next_step()

                agent_to_update_next = agent_to_update_next + 1

                if agent_to_update_next >= len(shuffled_list):
                    agent_to_update_next = 0

            runs = runs + 1
            
            my_world.id.stepSimulation()

        my_world.id.disconnect()

        return log_file, self.sim_failed
    
    def strain_test(self, time_steps):
        """
        Conduct a quick strain test with 2 robots to evaluate the tether's strain profile
        """
        self.reset_simulation()

        my_world = World(200, 200, self.time_step, self.gui_on)

        gradient_source = [100, 0]

        my_world.set_gradient_source(gradient_source)

        Agent.set_weights([0, 0, self.weight_gradient, 0])

        start_angles = [180, 180]

        initial_agent_positions = sims_utils.basic_starting_positions(self.unstretched_tether_length, 2, start_angles, [-2, (2 - 1) * self.unstretched_tether_length / 2, 0], "+y")

        goal_angles = [None, None]

        # populates the list of robot objects with agent objects
        for i in range(2):
            my_world.create_agent(initial_agent_positions[i], 0, radius = self.agent_radius, goal_delta = goal_angles[i], 
                            mass=self.agent_mass, height=self.agent_height, color=(0.5, 0.5, 1, 1), mu_static=self.agent_static_mu,
                            mu_dynamic=self.agent_dynamic_mu, max_velocity=self.agent_max_speed, drive_power=self.agent_drive_power)

        my_world.agent_list[1].failed = True # make sure agent doesn't try to move
        my_world.agent_list[1].fix_agent() # fix agent in place

        # populates the list of tether objects with tether objects
        my_world.create_and_anchor_tether(my_world.agent_list[0], my_world.agent_list[1], self.unstretched_tether_length, self.tether_youngs_modulus, self.tether_diameter, num_segments=10)

        my_world.display_axis_labels()
        
        log_file = f"data/strain_test.csv"
        runs = 0

        agent_to_update_next = 0

        shuffled_list = random.sample(my_world.agent_list, k=len(my_world.agent_list))

        # main simulation loop
        while my_world.id.isConnected() and runs <= time_steps:
            my_world.id.getCameraImage(320,200)

            strain = my_world.agent_list[0].tethers[1].get_strain()
            # area = math.pi * (self.tether_diameter / 2)**2
            # force = self.tether_youngs_modulus * area * strain
            force = utils.magnitude_of_vector(my_world.agent_list[0].get_net_force()[:2])

            csv_row = [runs, force, strain]
            sims_utils.log_to_csv(log_file, csv_row, header=["time step", "force", "strain"])

            for agent in shuffled_list:
                agent.sense_gradient(my_world.gradient_source)
                agent.sense_close_range(my_world.obj_list, sensing_mode=2)

            if runs % self.sensing_period == 0:
                for i in range(len(shuffled_list)):
                    if i == agent_to_update_next:
                        shuffled_list[i].set_next_step()

                agent_to_update_next = agent_to_update_next + 1

                if agent_to_update_next >= len(shuffled_list):
                    agent_to_update_next = 0

            runs = runs + 1
            
            my_world.id.stepSimulation()
        
        my_world.id.disconnect()

        return log_file, self.sim_failed
    
    def reset_simulation(self):
        """
        Resets the simulation state and flags, clearing any previous data and preparing for a new simulation run.
        """
        self.sim_failed = False
        self.debounce_count = 0
        # additional reset logic can be added here if needed
