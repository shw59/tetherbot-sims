"""
main.py

This file contains the main driver program for running the tetherbot PyBullet simulation.
"""

import pybullet as p
from simulations import Simulation
import simulation_utils as sims_utils
import time
import multiprocessing as mp
import datetime
import matplotlib.pyplot as plt
import numpy as np
import pickle

TIME_STEP = 1/240 # seconds
SENSING_PERIOD = 5 # number of while loop iterations that run before an agent position updates
LOGGING_PERIOD = 20 # number of while loop iterations that pass before data is written to a csv file

# vector weightings
ANGLE_WEIGHT = 15
STRAIN_WEIGHT = 300
GRADIENT_WEIGHT = 50
REPULSION_WEIGHT = 10

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

SIM_LOG_FILE = "data/logs/sim_log.csv"
SIM_LOG_HEADER = ["start time", "simulation type", "elapsed time (s)", "failed trials"]


def run_obstacle_simulations(sim_args, n, length_of_simulation, offsets, angles_to_try, number_of_trials, obst_position, obst_radius):
    """
    length_of_simulation: this number is an integer, and it is multiplied by the 
                        LOGGING_PERIOD to determine how long to run the while loop for
    """
    start_time = time.perf_counter()
    curr_time = datetime.datetime.now()

    args, gui_on = sim_args
    sim = Simulation(*args, gui_on=gui_on)
    list_of_file_names = []
    for t in range(number_of_trials):
        for a in angles_to_try:
            for o in offsets:
                list_of_file_names.append(sim.obstacle_avoidance(n, o, a, stop = length_of_simulation*LOGGING_PERIOD, trial = t + 1, obst_radius=obst_radius, obst_pos = obst_position))

    sims_utils.heat_map(number_of_trials, angles_to_try, offsets, LOGGING_PERIOD, obst_radius)
   
    end_time = time.perf_counter()

    elapsed_time = end_time - start_time

    sims_utils.log_to_csv(SIM_LOG_FILE, [curr_time, "obstacle avoidance", "total trials", "failed trials", elapsed_time], ["start time", "simulation type", "total trials", "failed trials", "elapsed time (s)"])

def run_tow_failed_agents_simulations(sim_args, n, num_runs, time_steps, agents_to_fail):
    """
    Runs a series of towing failed agents simulations. 

    n: Number of agents in simulation
    num_runs: Number of trials per failed agent
    agents_to_fail: List of agent numbers within n that are to fail
    """
    start_time = time.perf_counter()
    curr_time = datetime.datetime.now()

    args, gui_on = sim_args
    sim = Simulation(*args, gui_on=gui_on)
    csv_averages_list = []
    failed_trials = []
    for failed_agent_num in agents_to_fail:
        trial_list = []
        for trial in range(1, num_runs + 1):
            file_name, trial_failed = sim.tow_failed_agents_trial(n, trial, time_steps, failed_agent_num)
            if trial_failed:
                print("hi")
                failed_trials.append(trial_failed)
            else:
                trial_list.append(file_name)
        csv_averages_list.append(sims_utils.average_csv_trials(trial_list, f"data/tow_failed_agents_trialavg_agent{failed_agent_num}_failed.csv"))

    sims_utils.make_graph(csv_averages_list, "time step", ["failed agent x-position"], [f"agent {i} failed" for i in range(n)],
                          "Failed Agent x-Position vs Time Step", "Time Step", ["Failed Agent x-Position"], f"data/figures/towing_agents_graph_{datetime.datetime.now().date()}.png")

    end_time = time.perf_counter()

    elapsed_time = end_time - start_time

    sims_utils.log_to_csv(SIM_LOG_FILE, [curr_time, "towing failed agents", elapsed_time, failed_trials], SIM_LOG_HEADER)

def run_object_capture_simulations(sim_args, n, num_trials, time_steps, object_nums, offsets, maintain_line):
    """
    Runs a series of object-capture simulations.

    n: Number of agents
    num_trials: Number of trials for each object amount
    object_nums: List of object numbers to run for
    maintain_line: True if we want the agents to maintain a straight line, False otherwise
    """
    start_time = time.perf_counter()
    curr_time = datetime.datetime.now()

    args, gui_on = sim_args
    sim = Simulation(*args, gui_on=gui_on)
    failed_trials = []
    for offset in offsets:
        csv_averages_list = []
        for object_num in object_nums:
            trial_list = []
            for trial in range(1, num_trials + 1):
                file_name, trial_failed = sim.object_capture_trial(n, trial, time_steps, object_num, offset, maintain_line)
                if trial_failed:
                    failed_trials.append(file_name)
                else:
                    trial_list.append(file_name)
            csv_averages_list.append(sims_utils.average_csv_trials(trial_list, f"data/object_capture_maintain_line_{maintain_line}_trialavg_objects{object_num}_offset{offset}.csv"))

        sims_utils.make_graph(csv_averages_list, "time step", ["collective radius", "# of objects collected"], [f"{object_num} objects" for object_num in object_nums],
                              f"Collective Radius and # of Objects Collected vs Time Step, with Offset {offset}", "Time Step", ["Collective Radius", "# of Objects Collected"], f"data/figures/object_capture_maintain_line_{maintain_line}_offset{offset}_graph_{datetime.datetime.now().date()}.png")

    end_time = time.perf_counter()

    elapsed_time = end_time - start_time

    sims_utils.log_to_csv(SIM_LOG_FILE, [curr_time, f"object capture, maintain line {maintain_line}", elapsed_time, failed_trials], SIM_LOG_HEADER)

    return (csv_averages_list, "time step", ["collective radius", "# of objects collected"], [f"{object_num} objects" for object_num in object_nums],
            "Collective Radius and # of Objects Collected vs Time Step", "Time Step", ["Collective Radius", "# of Objects Collected"], f"data/figures/object_capture_maintain_line_{maintain_line}_graph_{datetime.datetime.now().date()}.png")

def run_storm_drain(sim_args):
    start_time = time.perf_counter()
    curr_time = datetime.datetime.now()
    
    args, gui_on = sim_args
    sim = Simulation(*args, gui_on=gui_on)
    sim.gui_on = True
    sim.storm_drain()

    end_time = time.perf_counter()

    elapsed_time = end_time - start_time

    sims_utils.log_to_csv(SIM_LOG_FILE, [curr_time, f"storm drain", "total trials", elapsed_time, ""], SIM_LOG_HEADER)

def main():
    """
    Is the function called when running the program. This function calls which ever function you want to test.
    """
    sim_args = (TIME_STEP, MASS, RADIUS, HEIGHT, MAX_SPEED, DRIVE_POWER, MU_STATIC, MU_DYNAMIC, 
                UNSTRETCHED_TETHER_LENGTH, YOUNGS_MODULUS, DIAMETER, ANGLE_WEIGHT, STRAIN_WEIGHT, GRADIENT_WEIGHT, REPULSION_WEIGHT, 
                SENSING_PERIOD, LOGGING_PERIOD)

    # run_storm_drain((sim_args, True))
    run_tow_failed_agents_simulations((sim_args, False), 5, 10, 10000, [0, 1, 2, 3, 4])
    # run_object_capture_simulations((sim_args, False), 9, 10, 10000, [5, 10, 30, 50], [0, 2, 4], False)
    # run_object_capture_simulations((sim_args, False), 9, 10, 10000, [5, 10, 30, 50], [0, 2, 4], True)
    # run_obstacle_simulations((sim_args, True), 3, 500, [-5*UNSTRETCHED_TETHER_LENGTH, -4*UNSTRETCHED_TETHER_LENGTH, -3*UNSTRETCHED_TETHER_LENGTH, -2*UNSTRETCHED_TETHER_LENGTH, -1*UNSTRETCHED_TETHER_LENGTH, 0, UNSTRETCHED_TETHER_LENGTH, 2*UNSTRETCHED_TETHER_LENGTH, 3*UNSTRETCHED_TETHER_LENGTH, 4*UNSTRETCHED_TETHER_LENGTH, 5*UNSTRETCHED_TETHER_LENGTH], [0], 3, [10,0], 4*UNSTRETCHED_TETHER_LENGTH)
    # sims_utils.make_3D_plot(["data/trial3_degree50_offset0.csv"], 9)


if __name__ == "__main__":
    main()