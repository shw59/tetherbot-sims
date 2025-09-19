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
ANGLE_WEIGHT = 15 # originally 15
STRAIN_WEIGHT = 500 # originally 500
GRADIENT_WEIGHT = 20 # originally 20
REPULSION_WEIGHT = 5

# jackal robot parameters
MASS = 17 # kg
RADIUS = 0.29 # m
HEIGHT = 0.25 # m
MAX_SPEED = 2 # m/s
DRIVE_POWER = 500 # watts
MU_STATIC = 1.25
MU_DYNAMIC = 0.9

# paracord 550 parameters
UNSTRETCHED_TETHER_LENGTH = 1.4
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
    list_of_averaged_trials = []
    failed_trials = []

    log_file = "data/success_of_averaged_trials_of_obstacle_avoidance.csv"

    log_header = ["Offsets", "Rate of Success", "Trial success"]

    # for t in range(number_of_trials):
    #     for a in angles_to_try:
    #         for o in offsets:
    #             list_of_file_names.append(sim.obstacle_avoidance(n, o, a, stop = length_of_simulation, trial = t + 1, obst_radius=obst_radius, obst_pos = obst_position))

    for a in angles_to_try:
        for o in offsets:
            list_of_file_in_a_trial = []
            avg_success = 0
            data = [str(o/UNSTRETCHED_TETHER_LENGTH)]
            mini_list = []
            for t in range(number_of_trials):
                file_name, trial_failed, while_loops = sim.obstacle_avoidance(n, o, a, stop = length_of_simulation, trial = t + 1, obst_radius=obst_radius, obst_pos = obst_position, a_weight=ANGLE_WEIGHT, s_weight=STRAIN_WEIGHT, g_weight=GRADIENT_WEIGHT, r_weight=REPULSION_WEIGHT)
                if trial_failed:
                    print("TRIAL FAILED")
                    failed_trials.append(trial_failed)
                else:
                    list_of_file_in_a_trial.append(file_name)
                    success = sims_utils.obstacle_avoidance_success([file_name], 1, 1, while_loops, LOGGING_PERIOD, n, obst_position, obst_radius)[0]
                    mini_list.append(success)
                    avg_success = avg_success + success

            avg_success = avg_success/number_of_trials

            # list_of_averaged_trials = [sims_utils.average_csv_trials(list_of_file_in_a_trial, f"data/average_for_offset_{o}.csv")]
            # success_of_offset = sims_utils.obstacle_avoidance_success(list_of_averaged_trials, 1, 1, length_of_simulation, LOGGING_PERIOD, n, obst_position, obst_radius)[0]

            data.append(avg_success)
            data.append(mini_list)

            sims_utils.log_to_csv(log_file, data, header=log_header)

    # sims_utils.heat_map(number_of_trials, angles_to_try, offsets, LOGGING_PERIOD, obst_radius)

    sims_utils.make_graph([log_file], "Offsets", ["Rate of Success"], title="Rate of success obstacle avoidance", x_label = "Number of tether lengths off center of obstacle", y_labels=["Rate of success"], file_name= f"data/figures/avoidance_graph_{datetime.datetime.now().date()}.png")
   
    end_time = time.perf_counter()

    elapsed_time = end_time - start_time

    sims_utils.log_to_csv(SIM_LOG_FILE, [curr_time, "obstacle avoidance", elapsed_time, failed_trials], SIM_LOG_HEADER)

    return True

def run_one_agent_follows_gradient(sim_args):
    start_time = time.perf_counter()
    curr_time = datetime.datetime.now()
    
    args, gui_on = sim_args
    sim = Simulation(*args, gui_on=gui_on)
    sim.one_agent_follows_gradient()

    end_time = time.perf_counter()

    elapsed_time = end_time - start_time

    sims_utils.log_to_csv(SIM_LOG_FILE, [curr_time, f"one agent follows gradient", elapsed_time, ""], SIM_LOG_HEADER)

    return True

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
                print("TRIAL FAILED")
                failed_trials.append(trial_failed)
            else:
                trial_list.append(file_name)
        if trial_list:
            csv_averages_list.append(sims_utils.average_csv_trials(trial_list, f"data/tow_failed_agents_trialavg_agent{failed_agent_num}_failed.csv"))
        else:
            return False
        
    sims_utils.make_graph(csv_averages_list, "time step", ["failed agent x-position"], [f"agent {i} failed" for i in range(n)],
                          "Failed Agent x-Position vs Time Step", "Time Step", ["Failed Agent x-Position"], f"data/figures/towing_agents_graph_{datetime.datetime.now().date()}.png")

    end_time = time.perf_counter()

    elapsed_time = end_time - start_time

    sims_utils.log_to_csv(SIM_LOG_FILE, [curr_time, "towing failed agents", elapsed_time, failed_trials], SIM_LOG_HEADER)

    return True

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
            if trial_list:
                csv_averages_list.append(sims_utils.average_csv_trials(trial_list, f"data/object_capture_maintain_line_{maintain_line}_trialavg_objects{object_num}_offset{offset}.csv"))
            else:
                return False

        sims_utils.make_graph(csv_averages_list, "time step", ["variance"], [f"{object_num} objects" for object_num in object_nums],
                              f"Tether Angle Variance vs Time Step, with Offset {offset}", "Time Step", ["Variance"], f"data/figures/object_capture_maintain_line_{maintain_line}_offset{offset}_graph_{datetime.datetime.now().date()}.png")

    end_time = time.perf_counter()

    elapsed_time = end_time - start_time

    sims_utils.log_to_csv(SIM_LOG_FILE, [curr_time, f"object capture, maintain line {maintain_line}", elapsed_time, failed_trials], SIM_LOG_HEADER)

    return True

def run_storm_drain(sim_args):
    start_time = time.perf_counter()
    curr_time = datetime.datetime.now()
    
    args, gui_on = sim_args
    sim = Simulation(*args, gui_on=gui_on)
    sim.storm_drain()

    end_time = time.perf_counter()

    elapsed_time = end_time - start_time

    sims_utils.log_to_csv(SIM_LOG_FILE, [curr_time, f"storm drain", elapsed_time, ""], SIM_LOG_HEADER)

    return True

# def run_strain_test(sim_args, time_steps):

#     start_time = time.perf_counter()
#     curr_time = datetime.datetime.now()

#     args, gui_on = sim_args
#     sim = Simulation(*args, gui_on=gui_on)
#     file_name, _ = sim.strain_test(time_steps)

#     sims_utils.make_graph([file_name], "strain", ["force"], ["Force"], "Tether Strain vs Force", "Tether Strain", ["Force"], f"data/figures/strain_profile{datetime.datetime.now().strftime("%H%M%S")}.png")
    
#     end_time = time.perf_counter()

#     elapsed_time = end_time - start_time
    
#     sims_utils.log_to_csv(SIM_LOG_FILE, [curr_time, f"strain test", elapsed_time, ""], SIM_LOG_HEADER)
    
#     return True
  
def run_building_plan(sim_args):
    start_time = time.perf_counter()
    curr_time = datetime.datetime.now()
    
    args, gui_on = sim_args
    sim = Simulation(*args, gui_on=gui_on)
    sim.gui_on = True
    sim.building_plan()
    
    end_time = time.perf_counter()
    
    elapsed_time = end_time - start_time
    
    sims_utils.log_to_csv(SIM_LOG_FILE, [curr_time, f"building plan", elapsed_time, ""], SIM_LOG_HEADER)
    
    return True

def run_w_to_m(sim_args, time_steps, num_trials):
    """
    Runs a series of W-to-M simulations.

    time_steps: Number of time steps to run for
    num_trials: Number of trials for each object amount
    """
    start_time = time.perf_counter()
    curr_time = datetime.datetime.now()

    args, gui_on = sim_args
    sim = Simulation(*args, gui_on=gui_on)
    failed_trials = []
    csv_averages_list = []
    trial_list = []
    for i in range (1, 4):
        for trial in range(1, num_trials + 1):
            file_name, trial_failed = sim.w_to_m(time_steps, trial, i)
            if trial_failed:
                failed_trials.append(file_name)
            else:
                trial_list.append(file_name)
        if trial_list:
            csv_averages_list.append(sims_utils.average_csv_trials(trial_list, f"data/w_to_m_trialavg_agent{i}.csv"))
        else:
            return False
    
    sims_utils.make_graph(csv_averages_list, "time step", [f"agent error"], [f"agent {i}" for i in range(1, 4)],
                            f"Error to Desired Tether Angle (W to M)", "Time Step", ["abs(error) (degrees)"], f"data/figures/w_to_m_graph_{datetime.datetime.now().date()}.png")

    end_time = time.perf_counter()

    elapsed_time = end_time - start_time

    sims_utils.log_to_csv(SIM_LOG_FILE, [curr_time, f"W to M", elapsed_time, failed_trials], SIM_LOG_HEADER)

    return True

def main():
    """
    Is the function called when running the program. This function calls which ever function you want to test.
    """
    sim_args = (TIME_STEP, MASS, RADIUS, HEIGHT, MAX_SPEED, DRIVE_POWER, MU_STATIC, MU_DYNAMIC, 
                UNSTRETCHED_TETHER_LENGTH, YOUNGS_MODULUS, DIAMETER, ANGLE_WEIGHT, STRAIN_WEIGHT, GRADIENT_WEIGHT, REPULSION_WEIGHT, 
                SENSING_PERIOD, LOGGING_PERIOD)
    
    # sim_args1 = (TIME_STEP, MASS, RADIUS, HEIGHT, MAX_SPEED, DRIVE_POWER, MU_STATIC, MU_DYNAMIC, 
    #             UNSTRETCHED_TETHER_LENGTH, YOUNGS_MODULUS, DIAMETER, 15, 500, 8, 5, 
    #             SENSING_PERIOD, LOGGING_PERIOD)

    # run_storm_drain((sim_args, True))
    # run_building_plan((sim_args1, True))

    # run_one_agent_follows_gradient((sim_args, True))

    # run_object_capture_simulations((sim_args, True), 9, 10, 15000, [25, 50, 75, 100], [0], False)
    # run_object_capture_simulations((sim_args, True), 9, 10, 15000, [100], [0], True)

    run_tow_failed_agents_simulations((sim_args, True), 5, 10, 15000, [2, 3, 4])

    # start = 0.0
    
    # num_of_offsets_plus_one = 21

    # not_sized_offsets = [3, 3.25, 3.5, 3.75, 4, 4.25, 4.5, 4.75, 5, 5.25, 5.5, 5.75, 6, 6.25, 6.5, 6.75, 7]

    # not_sized_offsets = []

    # for i in range(num_of_offsets_plus_one):
    #     not_sized_offsets.append(start)
    #     start = start + 0.25

    # offsets = []

    # for i in not_sized_offsets:
    #     offsets.append(i*UNSTRETCHED_TETHER_LENGTH)

    # offsets = [2.5*UNSTRETCHED_TETHER_LENGTH]

    # offsets = [0, 10*UNSTRETCHED_TETHER_LENGTH]
    
    # run_obstacle_simulations((sim_args, True), 9, 10000, offsets, [0], 5, [6,0], 4*UNSTRETCHED_TETHER_LENGTH)
    # sims_utils.make_3D_plot(["data/trial1_degree0_offset0.0.csv"], 9)

    # trials = 7
    
    # for t in range(1, trials + 1):
        # for i in not_sized_offsets:
            # name = "data/trial" + str(t) + "_degree0_offset" + str(i*UNSTRETCHED_TETHER_LENGTH) + ".csv"
            # print("Trial: " + str(t))
            # print("Offset: " + str(i))
            # result = sims_utils.obstacle_avoidance_success([name], 1, 1, 10000, LOGGING_PERIOD, 9, [6,0], 4*UNSTRETCHED_TETHER_LENGTH)[0]
            # print("Result: " + str(result))
            # print("\n")
    # sims_utils.make_3D_plot(["data/8-26-26_to_9-3-25/trial5_degree0_offset1.5.csv"], 9)

    # run_strain_test((sim_args, True), 500)
    # run_w_to_m((sim_args, False), 1500, 8)

if __name__ == "__main__":
    main()

