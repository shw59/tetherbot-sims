"""
simulation_utils.py

This file contains useful functions for generating and running simulations.
"""

import pybullet as p
import numpy as np
import matplotlib.pyplot as plt
import math
import random
import csv
import pandas as pd
import os

def basic_starting_positions(l_0, n, angles, starting_position, direction):
    """
    Calculates and returns a list the starting position based on the list of 
    desired starting angles and the unstretched length of the tether between 
    the agents. Builds the agents in the direction specified by the input parameter,
    of which there are four allowable directions.

    l_0: unstretched and uncompressed length of the tethers between each agent
    n: integer, number of agents
    starting_position: the position of the first agent, given as [x, y, z].
    direction: a string, can be "+x", "-x", "+y", or "-y", and will build the robots
               in the direction that is described in the string from the starting_position
    """

    if len(angles) != n:
        print("ANGLES LIST MUST MATCH THE NUMBER OF AGENTS.")
        exit(1)

    the_list = []

    for i in range(n):
        the_list.append([0])

    the_list[0] = starting_position

    if angles[0] is not None:
        if direction[1::] == "x":
            assumed_heading = math.atan2(1, 0)
        elif direction[1::] == "y":
            assumed_heading = math.atan2(0, 1)
        else:
            print("Please specify the direction you want to build the robots. Can be '+x', '-x', '+y', or '-y'")
            return the_list
        
        theta = assumed_heading + math.radians(angles[0])
        next_position = [starting_position[0] + l_0 * math.cos(theta), starting_position[1] + l_0 * math.sin(theta), 0]
        the_list[1] = next_position

    for i in range(1, n):
        if angles[i-1] is None:
            if direction == "+x":
                next_position = [the_list[i-1][0] + l_0, the_list[i-1][1], the_list[i-1][2]]
            elif direction == "-x":
                next_position = [the_list[i-1][0] - l_0, the_list[i-1][1], the_list[i-1][2]]
            elif direction == "+y":
                next_position = [the_list[i-1][0], the_list[i-1][1] + l_0, the_list[i-1][2]]
            elif direction == "-y":
                next_position = [the_list[i-1][0], the_list[i-1][1] - l_0, the_list[i-1][2]]
            else:
                print("Please specify the direction you want to build the robots. Can be '+x', '-x', '+y', or '-y'")
                return the_list
        elif i != 1:
            prev_theta = np.degrees(math.atan2(the_list[i-2][1] - the_list[i-1][1], the_list[i-2][0] - the_list[i-1][0]))
            new_theta = prev_theta + (360-angles[i-1])
            new_x = the_list[i-1][0] + l_0*np.cos(np.radians(new_theta))
            new_y = the_list[i-1][1] + l_0*np.sin(np.radians(new_theta))
            next_position = [new_x, new_y, 0]

        the_list[i] = next_position

    return the_list

def obstacle_avoidance_success(list_of_files, number_of_trials, number_of_runs_per_trial, number_of_while_runs, logging_period, n, obst_position, l_0):
    """
    This function returns a list that is number_of_angles*number_of_offsets long, 
    where each entry corresponds with the success rate of a particular 
    angle/offset pair, averaged over all of the trials of each particular pair.

    list_of_fils: a list of csv files, where each file corresponds with a specific
                  trial number of a particular angle/offset pair
    number_of_trials: an integer value that is the number of trials ran for each pair, which
                      is the same for all the pairs
    number_of_runs_per_trial: an integer, represents how many different angle/offest pairs
                              are ran for each trial. This is equal to 
                              the value of number_of_angles*number_of_offsets long
    number_of_while_runs: an integer number, represents the number of times the while loop
                          should loop
    logging_period: an integer, represents how often to gather and store the wanted data
    n: an integer, the number of agents in the simulation
    obst_position: the xy-coordinate of the obstacle
    l_0: the unstretched length of the tethers between each agent
    """

    # If the simulation doesn't run for long enough, increase the simulation time length
    if (number_of_while_runs-4*logging_period <= 0):

        print("Increase the number of while loop iterations please")
        return 0
    
    else:

        # The following snippet of code creates a list of lists of csv files 
        # named each_runs_trial, and these sublists are the results from different 
        # trials of a particular angle/offset pair. In other words, it groups 
        # the data from each angle/offset pair with their own trials, 
        # and places these groups in a list

        each_runs_trial = []

        for i in range(number_of_runs_per_trial):

            set_up = []
            for k in range(number_of_trials):
                set_up.append(list_of_files[i+k*number_of_runs_per_trial])
            each_runs_trial.append(set_up)


        success_list = []

        # Goes through each entry in each_runs_trial, where each entry
        # is a list of the varius trials that correspond to a particula 
        # angle/offset pair
        for i in range(len(each_runs_trial)):

            total = 0
            successful = 0

            # Goes through each trial, so this is the length of 
            # each list within each_runs_trial
            for k in range(number_of_trials):

                # In a particular angle/offset pair, opens the csv file
                # corresponding with trial k
                with open(each_runs_trial[i][k]) as csv_file:

                    csv_reader = csv.reader(csv_file, delimiter=',')
                    avg_velocity = 0
                    avg_distance_from_obstacle = 0
                    line_count = 0

                    # reads through each row of the csv file
                    for row in csv_reader:

                        # skips the first row, which only has headings and no data
                        if line_count != 0:

                            # checks to see if the number of runs of the while loop
                            # matches the number of runs that the data in this row
                            # was collected at. We only want velocity and distance
                            # data at the end of the simulation
                            if (int(row[0]) == (number_of_while_runs)):

                                # sums velocity and distance from obstacel of all agents
                                for m in range(n):
                                    avg_velocity = avg_velocity + float(row[(m+1)*3])
                                    avg_distance_from_obstacle = avg_distance_from_obstacle + math.sqrt(((obst_position[0]-float(row[(m*3)+1]))**2)+((obst_position[1]-float(row[(m*3)+2]))**2))

                        else:
                            line_count = line_count + 1

                    # if the average velocity is less than 0.3 and the average distance is less than
                    # 1.5 obstacles away, then we consider the test to have been unsuccessful because 
                    # the agents are moving very slowly and are not far from the obstacle
                    if (avg_velocity/n) <= 0.3 and ((avg_distance_from_obstacle/n) <= 1.5*l_0):
                        total = total + 1
                    else:
                        successful = successful + 1
                        total = total + 1

            success_list.append(successful/total)
        
        return success_list

def generate_circular_obstacle_course(world, n_obstacles, obstacle_size_range, course_radius):
    """
    Generates a circular obstacle within a specified radius with obstacles of specified size.

    world: the id of a world object
    n_obstactls: the number of obstacles to be created, an integer
    obstacle_size_range: the minimum and maximum obstacle diameter
    course_radius: radius of the course
    """

    for _ in range(n_obstacles):
        r = random.uniform(0, course_radius - (obstacle_size_range[1] / 2))
        theta = random.uniform(0, 2 * math.pi)
        x = r * math.cos(theta)
        y = r * math.sin(theta)
        sizes = random.uniform(obstacle_size_range[0], obstacle_size_range[1])
        no_overlap = True
        for obj in world.obj_list:
            if obj.label == "obstacle" and math.dist(obj.get_pose()[0], [x, y]) <= sizes * 2:
                no_overlap = True
                break
        if no_overlap:
            world.create_obstacle("hexagon", [x, y], length=sizes, width=sizes, color=(0, 1, 0, 1), fixed=True)
        
def angle_and_position_offset(n, angle, offset, seperation):
    """
    Returns 2D positions that are in a straight line that is some angle off of the 
    y-axis from the y-intercept position, and the y-intercept is determined based
    on the offset.

    n: positive integer of positions to generate
    angle: angle off of +y-axis towards +x-axis in degrees
    offset: value of the y-offset of the center of the line of agents about (0,0)
    seperation: value of unstretched tether between agents
    """
    positions = []

    for i in range(n):
        positions.append([0,0,0])

    bottom = 0.5*seperation*(1-n)
    positions[0] = [bottom, 0]

    for i in range(1, n):
        positions[i] = [bottom+seperation*(i), 0, 0]

    angle_off_positive_x = np.radians(90 - angle)

    for i in range(n):
        positions[i] = [round(np.cos(angle_off_positive_x)*positions[i][0], 5), round(np.sin(angle_off_positive_x)*positions[i][0], 5) + offset, 0]

    return positions

def generate_obstacles(world, corner_1, corner_2, num, shape, length, width, fixed):
    """
    Randomly generates obstacles within a box made by two corner positions.
    """
    for _ in range(num):
        x = random.uniform(corner_1[0], corner_2[0])
        y = random.uniform(corner_1[1], corner_2[1])
        no_overlap = False
        while not no_overlap:
            no_overlap = True
            x = random.uniform(corner_1[0], corner_2[0])
            y = random.uniform(corner_1[1], corner_2[1])
            no_overlap = True
            for obj in world.obj_list:
                if obj.label == "obstacle" and math.dist(obj.get_pose()[0], [x, y]) <= max(length, width) * 2:
                    no_overlap = False
                    break
        world.create_obstacle(shape, position=[x, y], length=length, width=width, fixed=fixed)

def log_to_csv(filename, data_row, header=None):
    """
    Logs a data row to a CSV file.
    If header is provided and the file is new, it writes the header first.
    """
    with open(filename, 'a', newline='') as csvfile:
        writer = csv.writer(csvfile)

        # Check if the file is empty to write header
        if csvfile.tell() == 0 and header:
            writer.writerow(header)

        writer.writerow(data_row)

def make_heat_map(data, angles, offsets, num_trials):
    """
    Saves an image of the succes rates for the obstacle avoidance simulation.

    data: a list of the success rates
    angles: a list where each entry in the list is an angle that was tested
    offsets: a list where each entry in the list is one of the offsets that was tested
    num_trials: an integer number of trials ran per angle/offset pair
    """

    # Need to make the list data into a list of list, where the rows correspond
    # to angles and the columns correspond to offsets, and the entries of the matrix
    # represent the success rates of a given angle/offset pair averaged over all trials
    organized_data = []

    for i in range(len(angles)):
        new = []
        for k in range(len(offsets)):
            new.append(data[i+k*(len(offsets)-1)])
        organized_data.append(new)

    # Plots this new list as a heat map with angles on the y-axis and offsets on the x-axis,
    # and the color of the heat map corresponds to the rate of success of the particul pair
    plt.imshow(organized_data, cmap='viridis', aspect='auto')
    plt.colorbar(label='Success Rate')
    plt.xticks(ticks=np.arange(len(offsets)), labels=offsets)
    plt.yticks(ticks=np.arange(len(angles)), labels=angles)
    plt.xlabel("Offset")
    plt.ylabel("Angle")
    plt.title("Rate of Success out of " + str((num_trials)) + " trials")
    # plt.savefig("data/figures/success_heat_map.png", format='png', dpi=300)  # dpi controls resolution
    plt.savefig("data/success_heat_map.png", format='png', dpi=300)  # dpi controls resolution
    plt.show()
    plt.close()  # Closes the figure to free memory
    return 0

def average_csv_trials(csv_files, output_filename, select_columns=0):
    """
    Averages all data points across different trials and saves to a new csv file

    csv_files (list of str): List of CSV file paths
    output_filename (str): Path to save the averaged result CSV file
    """
    
    data = {}
    for i in range(len(csv_files)):
        data[i] = pd.read_csv(f"data/test_runs/{csv_files[i]}", usecols=[i for i in range(select_columns)], index_col=False).reset_index(drop=True)

    df = pd.concat(data)

    # convert numeric columns to numeric (if necessary), errors to NaN
    df = df.apply(pd.to_numeric, errors='coerce')

    # average row-wise across trials (group by original row index)
    avg = df.groupby(level=1).mean()

    # save to csv
    avg.to_csv(f"data/{output_filename}", index=False)

    return avg

def make_graph(csv_files, x_column, y_column, labels=None, title="Tetherbot Plot",
                      x_label="x-axis", y_label="y-axis", file_name="graph.png"):
    """
    Plots the specified x and y columns from one or more CSV files.

    csv_files (list of str): Paths to the CSV files
    x_column (str): Name of the column to use as the x-axis
    y_column (str): Name of the column to use as the y-axis
    labels (list of str, optional): Labels for each CSV file's plot
    title (str, optional): Title of the plot
    xlabel (str, optional): Label for the x-axis
    ylabel (str, optional): Label for the y-axis
    file_name (str, optional): If provided, saves the figure with this file name
    """
    plt.figure(figsize=(10, 6))

    for i, file in enumerate(csv_files):
        df = pd.read_csv(file, index_col=False)
        plt.plot(df[x_column], df[y_column], label=labels[i])

    plt.title(title)
    plt.xlabel(x_label if x_label else x_column)
    plt.ylabel(y_label if y_label else y_column)
    plt.legend()
    plt.grid(True)
    plt.tight_layout()

    plt.savefig(f"data/figures/{file_name}", format='png')

    plt.show()