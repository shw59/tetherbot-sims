"""
simulation_utils.py

This file contains useful functions for generating and running simulations.
"""

import pybullet as p
import numpy as np
import matplotlib as plt
import math
import random
import csv

def basic_starting_positions(l_0, n, angles, starting_position, direction):
    """
    Calculates and returns a list the starting position based on the list of 
    desired starting angles and the unstretched length of the tether between 
    the agents. Builds the agents in the direction specified by the input parameter,
    of which there are four allowable directions.

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

def obstacle_avoidance_success(list_of_files, number_of_trials, number_of_runs_per_trial, number_of_while_runs):
    if (number_of_while_runs-4*LOGGING_PERIOD <= 0):
        print("Increase the number of while loop iterations please")

        return 0
    
    else:

        each_runs_trial = []

        for i in range(number_of_runs_per_trial):
            set_up = []
            for k in range(number_of_trials):
                set_up.append(list_of_files[i+k*number_of_runs_per_trial])
            each_runs_trial.append(set_up)

        success_list = []

        for i in range(len(each_runs_trial)):
            total = 0
            successful = 0
            for k in range(number_of_trials):
                with open(each_runs_trial[i][k]) as csv_file:
                    csv_reader = csv.reader(csv_file, delimiter=',')
                    initial_x = 0
                    initial_y = 0
                    final_x = 0
                    final_y = 0

                    line_count = 0

                    for row in csv_reader:
                        if line_count != 0:
                    
                            if (int(row[0]) == (number_of_while_runs*(0.8))):
                                initial_x = float(row[1])
                                initial_y = float(row[2])

                            if (int(row[0]) == (number_of_while_runs)):
                                final_x = float(row[1])
                                final_y = float(row[2])

                            line_count = line_count + 1
                        else:
                            line_count = line_count + 1

                    if (abs(final_x - initial_x) <= 0.3 and abs(final_y - initial_y) <= 0.3):
                        total = total + 1
                    else:
                        successful = successful + 1
                        total = total + 1

            success_list.append(successful/total)
        
        return success_list

def generate_circular_obstacle_course(world, n_obstacles, obstacle_size_range, course_radius):
    """
    Generates a circular obstacle within a specified radius with obstacles of specified size.
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
    n: number of positions to generate
    angle: angle off of +y-axis towards +x-axis in degrees
    offset: value of the y-offset of the center of the line of agents about (0,0)
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
    organized_data = []

    for i in range(len(angles)):
        new = []
        for k in range(len(offsets)):
            new.append(data[i+k*len(offsets)])
        organized_data.append(new)

    plt.imshow(organized_data, cmap='viridis', aspect='auto')
    plt.colorbar(label='Success Rate')
    plt.xticks(ticks=np.arange(len(offsets)), labels=offsets)
    plt.yticks(ticks=np.arange(len(angles)), labels=angles)
    plt.xlabel("Offset")
    plt.ylabel("Angle")
    plt.title("Rate of Success out of " + str((num_trials)) + " trials")
    plt.savefig("success_heat_map.jpg", format='jpg', dpi=300)  # dpi controls resolution
    plt.show()
    plt.close()  # Closes the figure to free memory
    return 0