"""
main.py

This file contains the main driver program for running the tetherbot PyBullet simulation.
"""

import pybullet as p
import pybullet_data
from world import World
from agent import Agent
from simulations import Simulation
import simulation_utils as sims_utils
import numpy as np
import math
import random
import csv
import time
import matplotlib.pyplot as plt

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
UNSTRETCHED_TETHER_LENGTH = 1.5
YOUNGS_MODULUS = 900e6
DIAMETER = 0.0019 # m


def main():
    """
    Is the function called when running the program. This function calls which ever function you want to test.
    """
    # create simulation instance and run simulations
    sim = Simulation(TIME_STEP, MASS, RADIUS, HEIGHT, MAX_SPEED, DRIVE_POWER, MU_STATIC, MU_DYNAMIC, 
                     UNSTRETCHED_TETHER_LENGTH, YOUNGS_MODULUS, DIAMETER, SENSING_PERIOD, LOGGING_PERIOD, gui_on=False)
    # sim.run_tow_failed_agents_simulations(n=5, num_runs=10, agents_to_fail=[0, 1, 2, 3, 4])
    # sim.run_object_capture_simulations(n=9, num_runs=10, object_nums=[5, 10, 30, 50], maintain_line=False)
    # sim.run_object_capture_simulations(n=9, num_runs=10, object_nums=[5, 10, 30, 50], maintain_line=True)
    sim.run_obstacle_simulations(n=9, l_0=UNSTRETCHED_TETHER_LENGTH, length_of_simulation=10, offsets=[0, 1], angles_to_try=[0], number_of_trials=2, obst_position = [4.5,0])
    # sim.gui_on = True
    # sim.storm_drain() # run and take screenshots

    # examples of averaging/plotting results
    # sims_utils.make_graph(["data/test_runs/tow_failed_agents_trial8_agent_2_failed.csv"], "time step", "agent 2 x-position", ["agent2"])
    # sims_utils.average_csv_trials(["object_capture_maintain_line_False_trial1_objects_5.csv", "object_capture_maintain_line_False_trial2_objects_5.csv"], "object_capture_maintain_line_false_trialavg_objects_5.csv", select_columns=3)

if __name__ == "__main__":
    main()