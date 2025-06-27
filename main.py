"""
main.py

This file contains the main driver program for running the tetherbot PyBullet simulation.
"""

import pybullet as p
from simulations import Simulation
import simulation_utils as sims_utils
import multiprocessing
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
UNSTRETCHED_TETHER_LENGTH = 1.5
YOUNGS_MODULUS = 900e6
DIAMETER = 0.0019 # m


def run_obstacle_simulations(sim_args, n, length_of_simulation, offsets, angles_to_try, number_of_trials, obst_position, obst_radius):
    """
    length_of_simulation: this number is an integer, and it is multiplied by the 
                        LOGGING_PERIOD to determine how long to run the while loop for
    """
    start_time = time.perf_counter()

    sim = Simulation(*sim_args)
    list_of_file_names = []
    for t in range(number_of_trials):
        for o in offsets:
            for a in angles_to_try:
                list_of_file_names.append(sim.obstacle_avoidance(n, o, a, stop = length_of_simulation*LOGGING_PERIOD, trial = t + 1, obst_radius=obst_radius, obst_pos = obst_position))

    data = sims_utils.obstacle_avoidance_success(list_of_files=list_of_file_names, number_of_trials=number_of_trials, number_of_runs_per_trial = len(offsets) * len(angles_to_try), number_of_while_runs=length_of_simulation*LOGGING_PERIOD, logging_period = LOGGING_PERIOD, n=n, obst_position = obst_position, obst_radius = obst_radius, l_0 = UNSTRETCHED_TETHER_LENGTH)

    sims_utils.make_heat_map(data=data, angles = angles_to_try, offsets = offsets, num_trials = number_of_trials)

    end_time = time.perf_counter()

    elapsed_time = end_time - start_time

    print("The time it took to run the obstacle avoidance simulation was " + str(elapsed_time) + " seconds.")

def run_tow_failed_agents_simulations(sim_args, n, num_runs, agents_to_fail):
    """
    Runs a series of towing failed agents simulations. 

    n: Number of agents in simulation
    num_runs: Number of trials per failed agent
    agents_to_fail: List of agent numbers within n that are to fail
    """
    start_time = time.perf_counter()

    sim = Simulation(*sim_args)
    for failed_agent_num in agents_to_fail:
        for trial in range(1, num_runs + 1):
            sim.tow_failed_agents_trial(n, trial, failed_agent_num)

    end_time = time.perf_counter()

    elapsed_time = end_time - start_time

    print("The time it took to run the towing simulation was " + str(elapsed_time) + " seconds.")

def run_object_capture_simulations(sim_args, n, num_runs, object_nums, maintain_line):
    """
    Runs a series of object-capture simulations.

    n: Number of agents
    num_runs: Number of trials for each object amount
    object_nums: List of object numbers to run for
    maintain_line: True if we want the agents to maintain a straight line, False otherwise
    """
    start_time = time.perf_counter()

    sim = Simulation(*sim_args)
    for object_num in object_nums:
        for trial in range(1, num_runs + 1):
            sim.object_capture_trial(n, trial, object_num, maintain_line)

    end_time = time.perf_counter()

    elapsed_time = end_time - start_time

    print("The time it took to run the object capture simulation was " + str(elapsed_time) + " seconds.")

def main():
    """
    Is the function called when running the program. This function calls which ever function you want to test.
    """
    sim_args = (TIME_STEP, MASS, RADIUS, HEIGHT, MAX_SPEED, DRIVE_POWER, MU_STATIC, MU_DYNAMIC, 
                UNSTRETCHED_TETHER_LENGTH, YOUNGS_MODULUS, DIAMETER, SENSING_PERIOD, LOGGING_PERIOD, False)
                
    processes = [
        # multiprocessing.Process(target=run_tow_failed_agents_simulations, args=(sim_args, 5, 10, [0, 1, 2, 3, 4])),
        # multiprocessing.Process(target=run_object_capture_simulations, args=(sim_args, 9, 10, [5, 10, 30, 50], False)),
        # multiprocessing.Process(target=run_object_capture_simulations, args=(sim_args, 9, 10, [5, 10, 30, 50], True)),
        multiprocessing.Process(target=run_obstacle_simulations, args=(sim_args, 9, 300, [-4*UNSTRETCHED_TETHER_LENGTH, -3*UNSTRETCHED_TETHER_LENGTH, -2*UNSTRETCHED_TETHER_LENGTH, -1*UNSTRETCHED_TETHER_LENGTH, 0, UNSTRETCHED_TETHER_LENGTH, 2*UNSTRETCHED_TETHER_LENGTH, 3*UNSTRETCHED_TETHER_LENGTH, 4*UNSTRETCHED_TETHER_LENGTH], [-15, -10, -5, 0, 5, 10, 15], 10, [10,0], 4*UNSTRETCHED_TETHER_LENGTH))
    ]

    for process in processes:
        process.start()

    for process in processes:
        process.join()

    # sim = Simulation(*sim_args)
    # sim.gui_on = True
    # sim.storm_drain() # run and take screenshots later (maybe automate)
    
    # # examples of averaging/plotting results
    # sims_utils.make_graph(["data/test_runs/tow_failed_agents_trial8_agent_2_failed.csv"], "time step", "agent 2 x-position", ["agent2"])
    # sims_utils.average_csv_trials(["object_capture_maintain_line_False_trial1_objects_5.csv", "object_capture_maintain_line_False_trial2_objects_5.csv"], "object_capture_maintain_line_false_trialavg_objects_5.csv", select_columns=3)

if __name__ == "__main__":
    main()