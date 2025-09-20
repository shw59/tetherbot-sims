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
import keyboard

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
        
        goal_angles = [None, 180, 180, 180, None]

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
        # my_world.create_obstacle("cube", [-13, -18], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        # my_world.create_obstacle("cube", [-14, -18], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
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

        # # big couch
        # my_world.create_obstacle("cube", [-10, -15], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        # my_world.create_obstacle("cube", [-10, -14], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        # my_world.create_obstacle("cube", [-10, -13], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        # my_world.create_obstacle("cube", [-10, -12], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        # my_world.create_obstacle("cube", [-10, -11], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        # my_world.create_obstacle("cube", [-10, -10], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)

        # my_world.create_obstacle("cube", [-9, -15], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        # my_world.create_obstacle("cube", [-9, -14], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        # my_world.create_obstacle("cube", [-9, -13], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        # my_world.create_obstacle("cube", [-9, -12], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        # my_world.create_obstacle("cube", [-9, -11], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        # my_world.create_obstacle("cube", [-9, -10], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)

        # my_world.create_obstacle("cube", [-8, -10], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        # my_world.create_obstacle("cube", [-8, -11], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        # my_world.create_obstacle("cube", [-7, -10], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        # my_world.create_obstacle("cube", [-7, -11], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        # my_world.create_obstacle("cube", [-6, -10], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        # my_world.create_obstacle("cube", [-6, -11], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        # my_world.create_obstacle("cube", [-5, -10], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        # my_world.create_obstacle("cube", [-5, -11], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        # my_world.create_obstacle("cube", [-4, -10], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        # my_world.create_obstacle("cube", [-4, -11], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        # my_world.create_obstacle("cube", [-3, -10], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        # my_world.create_obstacle("cube", [-3, -11], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        # my_world.create_obstacle("cube", [-2, -10], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)
        # my_world.create_obstacle("cube", [-2, -11], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=1)

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

        my_world.create_obstacle("hexagon", [-2,-10], length=5, width=5, color=(0, 0, 0, 1), fixed=True, height=1)

        # things to collect
        sims_utils.generate_obstacles(my_world, [16, 0], [25, 14], 5, "cylinder", 0.5, 0.5, 0.5, False)

        


        runs = 0

        agent_to_update_next = 0

        shuffled_list = random.sample(my_world.agent_list, k=len(my_world.agent_list))

        # main simulation loop
        while my_world.id.isConnected() and math.dist(my_world.agent_list[0].get_pose()[0], gradient) > 10:
            # if keyboard.is_pressed('s'):
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
                sims_utils.screenshot_gui(ss_filename=f"data/time_step_{runs}_building_plan_screenshot.png")

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
        
        n = 6

        bottom_position = -5
        top_position = 15
        right_position = 6
        left_position = -6
        tunnel_width = 2
        tunnel_length = 5
        length_of_tank = right_position - left_position
        doorway_length = length_of_tank/2

        size_of_block = 0.1
        blocks_per_unit = 1/size_of_block

        ramp_length = 2

        starter_length = n + 5

        # gradient = [(np.sqrt(2)*(length_of_tank/2))*((4/3)*((length_of_tank)/2)), (np.sqrt(2)*(length_of_tank))*((4/3)*(top_position-bottom_position))]
        gradient = [6, 30]

        my_world = World(120, 120, self.time_step, self.gui_on)

        my_world.create_obstacle("hexagon", gradient, length=1, width=1, color=(1, 0, 0, 1), fixed=True, height=0.5)

        my_world.set_gradient_source(gradient)

        Agent.set_weights([self.weight_angle, self.weight_strain, self.weight_gradient, self.weight_repulsion])

        angles = [None, 180, 180, 180, 180, None]

        pos = [left_position - tunnel_length - ramp_length - n - 2, bottom_position - ramp_length - int(tunnel_width/2), 0]
        
        initial_robot_positions = sims_utils.basic_starting_positions(self.unstretched_tether_length, n, angles, pos, "+x")
        
        goal_angles = [None, 160, 160, 160, 160, None]

        # populates the list of robot objects with robot objects
        for i in range(n):
             my_world.create_agent(initial_robot_positions[i], 0, radius = self.agent_radius, goal_delta = goal_angles[i], 
                                    mass=self.agent_mass, height=self.agent_height, color=(0.5, 0.5, 1, 1), mu_static=self.agent_static_mu,
                                    mu_dynamic=self.agent_dynamic_mu, max_velocity=self.agent_max_speed, drive_power=self.agent_drive_power)

            # if (i == 0) or (i == (n-1)):
                # my_world.create_agent(initial_robot_positions[i], 0, radius = self.agent_radius, goal_delta = goal_angles[i], 
                                    # mass=self.agent_mass, height=self.agent_height, color=(0.5, 0.5, 1, 1), mu_static=self.agent_static_mu,
                                    # mu_dynamic=self.agent_dynamic_mu, max_velocity=self.agent_max_speed, drive_power=self.agent_drive_power)
            # else:
                # my_world.create_agent(initial_robot_positions[i], 0, radius = self.agent_radius, goal_delta = goal_angles[i], 
                                    # mass=self.agent_mass, height=self.agent_height, color=(0.5, 0.5, 1, 1), mu_static=self.agent_static_mu,
                                    # mu_dynamic=self.agent_dynamic_mu, max_velocity=self.agent_max_speed, drive_power=self.agent_drive_power, dont_care_about_gradient=True)
                                    
        # populates the list of tether objects with tether objects
        for i in range(n-1):
            my_world.create_and_anchor_tether(my_world.agent_list[i], my_world.agent_list[i+1], self.unstretched_tether_length, self.tether_youngs_modulus, self.tether_diameter, num_segments = 10)


        
        # sides of sewer tank
        for i in range(0, int(blocks_per_unit*(top_position-bottom_position))):
            my_world.create_obstacle("cube", [left_position, size_of_block*i+bottom_position], length=size_of_block, width=size_of_block, color=(0, 0, 0, 1), fixed=True, height=0.5)
            my_world.create_obstacle("cube", [right_position, size_of_block*i+bottom_position], length=size_of_block, width=size_of_block, color=(0, 0, 0, 1), fixed=True, height=0.5)

        # top of the sewer tank
        
        for i in range(0, int((blocks_per_unit*(int(length_of_tank/2)-int(doorway_length/2))))):
            my_world.create_obstacle("cube", [size_of_block*i + left_position, top_position + size_of_block*i], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
            my_world.create_obstacle("cube", [right_position - size_of_block*i, top_position + size_of_block*i], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        
        # bottom of the sewer tank
        for i in range(0, int((blocks_per_unit*(right_position - left_position)))):
            my_world.create_obstacle("cube", [size_of_block*i + left_position, bottom_position - tunnel_width], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)

        for i in range(0, int(blocks_per_unit*tunnel_width)):
            if i != 0 or i != (int(blocks_per_unit*tunnel_width) - 1):
                my_world.create_obstacle("cube", [right_position, bottom_position - tunnel_width + i*size_of_block], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        
        for i in range(0, -1*int(blocks_per_unit*(tunnel_length - 1)), -1):
            if i != 0:
                my_world.create_obstacle("cube", [size_of_block*i + left_position, bottom_position], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
                my_world.create_obstacle("cube", [size_of_block*i + left_position, bottom_position - tunnel_width], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        
        for i in range(0, -int(blocks_per_unit*ramp_length), -1):
            my_world.create_obstacle("cube", [size_of_block*i + (left_position - tunnel_length + 1 ), size_of_block*i + bottom_position], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
            my_world.create_obstacle("cube", [size_of_block*i + (left_position-tunnel_length + 1), size_of_block*i +bottom_position - tunnel_width], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
        
        for i in range(0, -int(blocks_per_unit*starter_length), -1):
            my_world.create_obstacle("cube", [size_of_block*i + (left_position-tunnel_length-ramp_length + 1), bottom_position - tunnel_width - ramp_length], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)
            my_world.create_obstacle("cube", [size_of_block*i + (left_position-tunnel_length-ramp_length + 1), bottom_position  - ramp_length], length=1, width=1, color=(0, 0, 0, 1), fixed=True, height=0.5)

        sims_utils.generate_obstacles(my_world, [left_position, bottom_position - tunnel_width], [right_position, top_position], 150, "cylinder", 0.25, 0.25, self.agent_height, False)
        
        runs = 0

        agent_to_update_next = 0

        shuffled_list = random.sample(my_world.agent_list, k=len(my_world.agent_list))

        set_up_runs = 200
        is_setting_up = True

        # main simulation loop
        while my_world.id.isConnected() and math.dist(my_world.agent_list[0].get_pose()[0], gradient) > 10:
            while is_setting_up:
                # if keyboard.is_pressed('q'):
                my_world.id.getCameraImage(320,200)
                # print(runs)
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
            # if runs % 1000 == 0:
            #     sims_utils.screenshot_gui(ss_filename=f"data/time_step_{runs}_storm_drain_screenshot.png")
            # if self.sim_failed:
            #     break
                runs = runs + 1
                if runs > set_up_runs:
                    is_setting_up = False
                    Agent.set_weights([self.weight_angle, self.weight_strain, self.weight_gradient, self.weight_repulsion])
                    runs = 0
                my_world.id.stepSimulation()
            my_world.id.getCameraImage(320,200)
            # print(runs)
            self.debounce_count = 0
            # if runs % self.logging_period == 0:
            for agent in shuffled_list:
            #     # if agent.tethers[0] is not None:
            #         # print(agent.tethers[0].get_strain())
            #     # else:
            #         # print(agent.tethers[1].get_strain())
                if runs > Simulation.run_debounce and agent.is_tether_slack():
                    self.debounce_count += 1
            #     # else:
            #     #     self.debounce_count = 0
                if runs%Simulation.run_debounce == 0:
                    if self.old_debounce_count >= Simulation.debounce_threshold and self.debounce_count >= Simulation.debounce_threshold:
                        self.sim_failed = True
                        break
                    self.old_debounce_count = self.debounce_count
            #     # if self.debounce_count >= Simulation.debounce_threshold:
            #     #     self.sim_failed = True
            #     #     break
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
            # # if runs % 1000 == 0:
            # #     sims_utils.screenshot_gui(ss_filename=f"data/time_step_{runs}_storm_drain_screenshot.png")
            # # if self.sim_failed:
            # #     break
            runs = runs + 1
            # if runs > set_up_runs:
            #     is_setting_up = False
            #     Agent.set_weights([self.weight_angle, self.weight_strain, self.weight_gradient, self.weight_repulsion])
            #     runs = 0
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

        # start = [10.66435,2.61832,1.7178015867129035,11.69837,4.41912,0.008455429980073625,13.7678,5.06304,0.0026835581232114874,13.78616,7.08026,0.0011685954320433077,15.71217,6.6164,0.0009300594955369165,17.00229,8.24761,0.1086975315872433,17.82913,6.40982,0.009906272248240856,19.75934,6.63694,0.7376781911783291,22.06267,6.86474,0.21756226673416776]
        
        # sets = int(len(start)/3)

        # pos = []

        # for i in range(sets):
        #     pos.append([start[i*3], start[(i*3)+1], 0])
        
        initial_agent_positions = sims_utils.angle_and_position_offset(n, angle_off_y, y_offset, self.unstretched_tether_length)

        # initial_agent_positions = pos


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

        # my_world.display_axis_labels()
        
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

            stop_while_loop = False

            if runs % self.logging_period == 0:
                min_x = my_world.agent_list[0].get_pose()[0][0]
                data = [runs]
                for agent in my_world.agent_list:
                    data.append(round(agent.get_pose()[0][0], 5))
                    data.append(round(agent.get_pose()[0][1], 5))
                    data.append(agent.get_velocity())

                    if agent.get_pose()[0][0] < min_x:
                        min_x = agent.get_pose()[0][0]
                    

                # data.append(True)
            
                # sims_utils.log_to_csv(log_file, data, header=log_header)

                if min_x >= obst_pos[0]+obst_radius:
                    stop_while_loop = True

            if self.sim_failed:
                break

            if stop_while_loop:
                break

            # if runs % 1000 == 0:
            #     sims_utils.screenshot_gui(ss_filename=f"data/recordings_valid/time_step_{runs}_obstacle_screenshot.png")

            runs = runs + 1
            
            my_world.id.stepSimulation()

        my_world.id.disconnect()

        return log_file, self.sim_failed, runs


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

        sims_utils.generate_obstacles(my_world, [-75, -30], [-90, -2], 200, "cylinder", 0.5, 0.5, False)


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

        update_cycles_to_weight = 0
        
        update_cycles_weighted = 0

        # shuffled_list = random.sample(my_world.agent_list, k=len(my_world.agent_list))

        # time.sleep(5)

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

        # my_world.display_axis_labels()
        
        log_file = f"data/tow_failed_agents_trial{trial_num}_agent{failed_agent_num}_failed.csv"
        runs = 0

        agent_to_update_next = 0

        shuffled_list = random.sample(my_world.agent_list, k=len(my_world.agent_list))

        # number of runs to get the collective stretched out
        set_up_runs = 100
        is_setting_up = True

        # main simulation loop
        while my_world.id.isConnected() and runs <= time_steps:
            # set-up phase
            while is_setting_up:
                if keyboard.is_pressed('q'):
                    my_world.id.getCameraImage(320,200)

                    print(runs)

                    for agent in shuffled_list:
                        # if agent.tethers[0] is not None:
                            # print(agent.tethers[0].get_strain())
                        # if agent.tethers[1] is not None:
                            # print(agent.tethers[1].get_strain())
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

                    if runs > set_up_runs:
                        is_setting_up = False
                        # set the experimental parameters
                        Agent.set_weights([self.weight_angle, self.weight_strain, self.weight_gradient, self.weight_repulsion])

                        runs = 0
                    
                    my_world.id.stepSimulation()
            # if keyboard.is_pressed('q'):
            my_world.id.getCameraImage(320,200)

            print(runs)

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

        Agent.set_weights([0, self.weight_strain, 0, self.weight_repulsion])

        start_angles = [None] + [180] * (n - 2) + [None]
        
        initial_agent_positions = sims_utils.basic_starting_positions(self.unstretched_tether_length, n, start_angles, [-2, -(n - 1) * self.unstretched_tether_length / 2 + offset, 0], "+y")

        goal_angles = start_angles[:]

        # populates the list of robot objects with agent objects
        for i in range(n):
            my_world.create_agent(initial_agent_positions[i], 0, radius = self.agent_radius, goal_delta = goal_angles[i], 
                                  mass=self.agent_mass, height=self.agent_height, color=(0.5, 0.5, 1, 1), mu_static=self.agent_static_mu,
                                  mu_dynamic=self.agent_dynamic_mu, max_velocity=self.agent_max_speed, drive_power=self.agent_drive_power)

        # populates the list of tether objects with tether objects
        for i in range(n-1):
            my_world.create_and_anchor_tether(my_world.agent_list[i], my_world.agent_list[i+1], self.unstretched_tether_length, self.tether_youngs_modulus, self.tether_diameter, num_segments=10)

        sims_utils.generate_obstacles(my_world, [0, -6], [4, 6], num_objects, "cylinder", 0.25, 0.25, self.agent_height, False)

        # my_world.display_axis_labels()
        
        log_file = f"data/object_capture_maintain_line_{maintain_line}_trial{trial_num}_objects{num_objects}_offset{offset}.csv"
        runs = 0
        agent_to_update_next = 0
        shuffled_list = random.sample(my_world.agent_list, k=len(my_world.agent_list))

        obj_collected = 0

        collective_radius_avgs = []
        variance = []

        # number of runs to get the collective stretched out
        set_up_runs = 800
        is_setting_up = True

        # main simulation loop
        while my_world.id.isConnected() and runs <= time_steps:
            # set-up phase
            while is_setting_up:
                if keyboard.is_pressed('q'):
                    my_world.id.getCameraImage(320,200)

                    print(runs)

                    for agent in shuffled_list:
                        # if agent.tethers[0] is not None:
                            # print(agent.tethers[0].get_strain())
                        # if agent.tethers[1] is not None:
                            # print(agent.tethers[1].get_strain())
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
                                shuffled_list[i].set_next_step_revised()
                                if runs < set_up_runs:
                                    for agent in my_world.agent_list:
                                        agent.next_position = [agent.get_pose()[0][0], agent.next_position[1]]
                                shuffled_list[i].move_to(shuffled_list[i].max_force)

                        agent_to_update_next = agent_to_update_next + 1

                        if agent_to_update_next >= len(shuffled_list):
                            agent_to_update_next = 0

                    runs = runs + 1

                    if runs > set_up_runs:
                        is_setting_up = False
                        # set the experimental parameters
                        if maintain_line:
                            Agent.set_weights([self.weight_angle, self.weight_strain, self.weight_gradient, self.weight_repulsion])
                        else:
                            Agent.set_weights([0, self.weight_strain, self.weight_gradient, self.weight_repulsion])

                        runs = 0
                    
                    my_world.id.stepSimulation()
            
            # actual experiment begins
            # if keyboard.is_pressed('q'): # step through simulation
            self.debounce_count = 0

            print(runs)

            my_world.id.getCameraImage(320,200)

            if runs % self.logging_period == 0:
                for obj in my_world.obj_list:
                    if obj.label == "obstacle":
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
                
                # get agent positions to put in csv if we want
                agent_pos = [agent.get_pose()[0] for agent in my_world.agent_list]

                # sliding window average (tether angle variance between agents)
                agent_deltas = [agent.get_delta() for agent in my_world.agent_list[1:n-2]]
                variance.append(np.var(agent_deltas))

                if len(variance) > 25:
                    variance.pop(0)

                avg_variance = np.mean(variance)

                csv_row = [runs, avg_variance, obj_collected] + agent_deltas
                sims_utils.log_to_csv(log_file, csv_row, header=["time step", "variance", "# of objects collected", "tether angles"] + ["" for _ in range(n * 2 - 1)])

            for agent in shuffled_list:
                # if agent.tethers[0] is not None:
                    # print(agent.tethers[0].get_strain())
                # if agent.tethers[1] is not None:
                    # print(agent.tethers[1].get_strain())
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
                        shuffled_list[i].move_to(shuffled_list[i].max_force)

                agent_to_update_next = agent_to_update_next + 1

                if agent_to_update_next >= len(shuffled_list):
                    agent_to_update_next = 0

            runs = runs + 1
            
            my_world.id.stepSimulation()

        my_world.id.disconnect()

        return log_file, self.sim_failed
    
    def w_to_m(self, time_steps, trial_num, agent_num):
        """
        Recreates the W-to-M hardware experiment in simulation for comparison purposes.
        """
        n = 5

        self.reset_simulation()

        my_world = World(150, 150, self.time_step, self.gui_on)

        Agent.set_weights([self.weight_angle, self.weight_strain, 0, 0])

        start_angles = [45, 90, 270, 90, 45]
        
        initial_agent_positions = sims_utils.basic_starting_positions(self.unstretched_tether_length, n, start_angles, [-2, -(n - 1) * self.unstretched_tether_length / 2, 0], "+y")

        goal_angles = [None, 270, 90, 270, None]

        # populates the list of robot objects with agent objects
        for i in range(n):
            my_world.create_agent(initial_agent_positions[i], 0, radius = self.agent_radius, goal_delta = goal_angles[i], 
                                  mass=self.agent_mass, height=self.agent_height, color=(0.5, 0.5, 1, 1), mu_static=self.agent_static_mu,
                                  mu_dynamic=self.agent_dynamic_mu, max_velocity=self.agent_max_speed, drive_power=self.agent_drive_power)

        # populates the list of tether objects with tether objects
        for i in range(n-1):
            my_world.create_and_anchor_tether(my_world.agent_list[i], my_world.agent_list[i+1], self.unstretched_tether_length, self.tether_youngs_modulus, self.tether_diameter, num_segments=10)

        my_world.display_axis_labels()
        
        log_file = f"data/w_to_m_trial{trial_num}_agent{agent_num}.csv"
        runs = 0
        agent_to_update_next = 0
        shuffled_list = random.sample(my_world.agent_list, k=len(my_world.agent_list))

        # main simulation loop
        while my_world.id.isConnected() and runs <= time_steps:

            self.debounce_count = 0

            my_world.id.getCameraImage(320,200)

            if runs % self.logging_period == 0:
                csv_row = [runs]
                for i in range(len(my_world.agent_list)):
                    if i==agent_num:
                        agent = my_world.agent_list[i]
                        if None not in agent.tethers:
                            curr_angle = agent.get_delta()
                            goal_angle = agent.desired_tether_angle
                            err = abs(goal_angle - curr_angle)
                            csv_row.append(err)
                sims_utils.log_to_csv(log_file, csv_row, header=["time step", "agent error"])

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
