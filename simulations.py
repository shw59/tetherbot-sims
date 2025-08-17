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

    def obstacle_avoidance(self, n, y_offset, angle_off_y, a_weight = 10, s_weight = 15, g_weight = 10, r_weight = 3, gradient = [30,0], obst_pos = [6,0], obst_radius = 1, obst_height = 1, obst_type = "hexagon", stop=2000, trial = 0):
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
                data.append(True)
            
                sims_utils.log_to_csv(log_file, data, header=log_header)

            if self.sim_failed:
                break

            runs = runs + 1
            
            my_world.id.stepSimulation()

        my_world.id.disconnect()

        return log_file

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

        sims_utils.generate_obstacles(my_world, [0, -2], [3, 2], num_objects, "cylinder", 0.1, 0.1, False)

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
    
    def reset_simulation(self):
        """
        Resets the simulation state and flags, clearing any previous data and preparing for a new simulation run.
        """
        self.sim_failed = False
        self.debounce_count = 0
        # additional reset logic can be added here if needed
