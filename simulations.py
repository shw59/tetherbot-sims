"""
simulations.py

This file contains all experimental simulation functions that we are collecting data from.
"""

import pybullet as p
import pybullet_data
from world import World
from agent import Agent
import utils
import simulation_utils as sims_utils
import numpy as np
import math
import random

class Simulation:
    def __init__(self, time_step, agent_mass, agent_radius, agent_height, agent_max_speed, agent_drive_power, agent_static_mu, agent_dynamic_mu, 
                 unstretched_tether_length, tether_youngs_modulus, tether_diameter, sensing_period, logging_period, gui_on):
        """
        Initializes a simulation object with the given attributes that apply to all simulations run from that instance.
        """
        self.time_step = time_step
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
        self.sensing_period = sensing_period
        self.logging_period = logging_period
        self.gui_on = gui_on

    def storm_drain(self):
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

        my_world = World(100, 100, self.time_step, self.gui_on)

        my_world.set_gradient_source(gradient)

        Agent.set_weights([a_weight, s_weight, g_weight, r_weight])

        angles = [None, 180, 180, 180, 180, 180, 180, 180, None]
        
        initial_robot_positions = sims_utils.basic_starting_positions(self.unstretched_tether_length, n, angles, [-30,-7,0], "+x")
        
        goal_angles = [None, 180, 180, 180, 180, 180, 180, 180, None]

        # populates the list of robot objects with robot objects
        for i in range(n):
            my_world.create_agent(initial_robot_positions[i], 0, radius = self.agent_radius, goal_delta = goal_angles[i], 
                                  mass=self.agent_mass, height=self.agent_height, color=(1, 0, 0, 1), mu_static=self.agent_static_mu,
                                  mu_dynamic=self.agent_dynamic_mu, max_velocity=self.agent_max_speed, drive_power=self.agent_drive_power)

        # populates the list of tether objects with tether objects
        for i in range(n-1):
            my_world.create_and_anchor_tether(my_world.agent_list[i], my_world.agent_list[i+1], self.unstretched_tether_length, self.tether_youngs_modulus, self.tether_diameter, num_segments = 10)

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

        my_world.display_axis_labels()
        
        runs = 0

        agent_to_update_next = 0

        shuffled_list = random.sample(my_world.agent_list, k=len(my_world.agent_list))

        # main simulation loop
        while my_world.id.isConnected():
            my_world.id.getCameraImage(320,200)

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

    def obstacle_avoidance(self, n, l_0, y_offset, angle_off_y, a_weight = 10, s_weight = 15, g_weight = 10, r_weight = 3, gradient = [20,0], obst_pos = [6,0], obst_radius = 1, obst_height = 1, obst_type = "hexagon", stop=2000, trial = 0):
        """
        Generates a very simple formation of agents in order to test the hysteresis.
        """

        my_world = World(200, 200, self.time_step, gui_on=self.gui_on)

        my_world.set_gradient_source(gradient)

        Agent.set_weights([a_weight, s_weight, g_weight, r_weight])
        
        initial_agent_positions = sims_utils.angle_and_position_offset(n, angle_off_y, y_offset, l_0)

        goals = []
        for i in range(n):
            if i == 0 or i == (n-1):
                goals.append(None)
            else:
                goals.append(180)

        # populates the list of robot objects with agent objects
        for i in range(n):
            my_world.create_agent(initial_agent_positions[i], 0, radius = self.agent_radius, goal_delta = goals[i], 
                                  mass=self.agent_mass, height=self.agent_height, color=(1, 0, 0, 1), mu_static=self.agent_static_mu,
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
        while (runs <= stop) and (p.isConnected()):
            if runs%30:
                p.getCameraImage(320,200)

            for agent in shuffled_list:
                agent.sense_gradient(my_world.gradient_source)
                agent.sense_close_range(my_world.obj_list, sensing_mode=2)

            if runs % self.sensing_period == 0:
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

            if runs % self.logging_period == 0:
                data = [runs]
                for agent in my_world.agent_list:
                    data.append(round(agent.get_pose()[0][0], 5))
                    data.append(round(agent.get_pose()[0][1], 5))
                    data.append(agent.get_velocity())
            
                sims_utils.log_to_csv(log_file, data, header=log_header)

            runs = runs + 1
            
            p.stepSimulation()

        p.disconnect()

        return log_file

    def tow_failed_agents_trial(self, n, trial_num, failed_agent_num):
        """
        This experiment takes a group of n agents in a W formation and causes one of them to fail at t = 100.
        The other agents attempt to tow the failed agent as the collective advances towards the gradient source goal.
        """
        my_world = World(50, 10, self.time_step, self.gui_on)

        a_weight = 8 # angle vector weighting
        s_weight = 5 # strain vector weighting
        g_weight = 10 # gradient vector weighting
        r_weight = 4 # repulsion vector weighting

        gradient_source = [100, 0]

        my_world.set_gradient_source(gradient_source)

        Agent.set_weights([a_weight, s_weight, g_weight, r_weight])

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
                                  mass=self.agent_mass, height=self.agent_height, color=(1, 0, 0, 1), mu_static=self.agent_static_mu,
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
        while my_world.id.isConnected() and runs < 5000:
            my_world.id.getCameraImage(320,200)

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
                sims_utils.log_to_csv(log_file, csv_row, header=["time step", f"agent {failed_agent_num} x-position", "formation velocity", "other agent x-positions"])

            if runs == 100:
                my_world.agent_list[failed_agent_num].failed = True

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

        return log_file

    def object_capture_trial(self, n, trial_num, num_objects, maintain_line):
        """
        This experiment takes a group of n agents and places them in a line. The agents then attempt to collect randomly placed movable obstacles.
        The collective may either attempt to maintain a straight line or have no goal angle.
        """
        my_world = World(50, 20, self.time_step, self.gui_on)

        a_weight = 100 # angle vector weighting
        s_weight = 10 # strain vector weighting
        g_weight = 4 # gradient vector weighting
        r_weight = 3 # repulsion vector weighting

        gradient_source = [100, 0]

        my_world.set_gradient_source(gradient_source)

        Agent.set_weights([a_weight, s_weight, g_weight, r_weight])

        start_angles = [None] + [180] * (n - 2) + [None]
        
        initial_agent_positions = sims_utils.basic_starting_positions(self.unstretched_tether_length, n, start_angles, [-2, -(n - 1) * self.unstretched_tether_length / 2, 0], "+y")

        if maintain_line:
            goal_angles = start_angles[:]
        else:
            goal_angles = [None] * n

        # populates the list of robot objects with agent objects
        for i in range(n):
            my_world.create_agent(initial_agent_positions[i], 0, radius = self.agent_radius, goal_delta = goal_angles[i], 
                                  mass=self.agent_mass, height=self.agent_height, color=(1, 0, 0, 1), mu_static=self.agent_static_mu,
                                  mu_dynamic=self.agent_dynamic_mu, max_velocity=self.agent_max_speed, drive_power=self.agent_drive_power)

        # populates the list of tether objects with tether objects
        for i in range(n-1):
            my_world.create_and_anchor_tether(my_world.agent_list[i], my_world.agent_list[i+1], self.unstretched_tether_length, self.tether_youngs_modulus, self.tether_diameter, num_segments=10)

        sims_utils.generate_obstacles(my_world, [0, -2], [3, 2], num_objects, "cylinder", 0.1, 0.1, False)

        my_world.display_axis_labels()
        
        log_file =f"data/object_capture_maintain_line_{maintain_line}_trial{trial_num}_objects{num_objects}.csv"
        runs = 0
        agent_to_update_next = 0
        shuffled_list = random.sample(my_world.agent_list, k=len(my_world.agent_list))

        obj_collected = 0

        # main simulation loop
        while my_world.id.isConnected() and runs < 5000:
            my_world.id.getCameraImage(320,200)

            if runs % self.logging_period == 0:
                obstacle_pos = []
                for obj in my_world.obj_list:
                    if obj.label == "obstacle":
                        obstacle_pos.append(obj.get_pose()[0])
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
                collective_radius = utils.get_collective_radius(agent_pos)

                csv_row = [runs, collective_radius, obj_collected] + agent_pos + obstacle_pos
                sims_utils.log_to_csv(log_file, csv_row, header=["time step", "collective radius", "# of objects collected", "agent positions", "", "", "", "", "", "", "", "", "", "obstacle positions"])

            for agent in shuffled_list:
                agent.sense_gradient(my_world.gradient_source)
                agent.sense_close_range(my_world.obj_list, sensing_mode=1)

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

        return log_file

    def run_obstacle_simulations(self, n, l_0, length_of_simulation, offsets, angles_to_try, number_of_trials, obst_position):
        """
        length_of_simulation: this number is an integer, and it is multiplied by the 
                            LOGGING_PERIOD to determine how long to run the while loop for
        """
        list_of_file_names = []
        for t in range(number_of_trials):
            for o in offsets:
                for a in angles_to_try:
                    list_of_file_names.append(self.obstacle_avoidance(n, l_0, o, a, stop = length_of_simulation*self.logging_period, trial = t + 1, obst_radius=4*l_0, obst_pos = obst_position))

        data = sims_utils.obstacle_avoidance_success(list_of_files=list_of_file_names, number_of_trials=number_of_trials, number_of_runs_per_trial = len(offsets) * len(angles_to_try), number_of_while_runs=length_of_simulation*self.logging_period, logging_period = self.logging_period, n=n, obst_position = obst_position, l_0 = l_0)

        sims_utils.make_heat_map(data=data, angles = angles_to_try, offsets = offsets, num_trials = number_of_trials)

    def run_tow_failed_agents_simulations(self, n, num_runs, agents_to_fail):
        """
        Runs a series of towing failed agents simulations. 

        n: Number of agents in simulation
        num_runs: Number of trials per failed agent
        agents_to_fail: List of agent numbers within n that are to fail
        """
        for failed_agent_num in agents_to_fail:
            for trial in range(1, num_runs + 1):
                self.tow_failed_agents_trial(n, trial, failed_agent_num)

    def run_object_capture_simulations(self, n, num_runs, object_nums, maintain_line):
        """
        Runs a series of object-capture simulations.

        n: Number of agents
        num_runs: Number of trials for each object amount
        object_nums: List of object numbers to run for
        maintain_line: True if we want the agents to maintain a straight line, False otherwise
        """
        for object_num in object_nums:
            for trial in range(1, num_runs + 1):
                self.object_capture_trial(n, trial, object_num, maintain_line)