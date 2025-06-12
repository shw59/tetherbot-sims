"""
main.py

This file contains the main driver program for running the tetherbot PyBullet simulation.
"""

import pybullet as p
import pybullet_data
from world import World
from agent import Agent

GRAVITY_Z = -9.81
HEIGHT = 0.01
TIME_STEP = 1./240.

def main():

    my_world = World(20, 20, TIME_STEP)

    # set initial object positions
    # initial_robot_positions = set_straight_line(N, l_0)
    
    # initial_robot_positions = [[0, 0, HEIGHT],
    #                            [0, 1, HEIGHT],
    #                            [1, 1, HEIGHT]]
    # # initial_robot_positions = [[1,1,height]]
    # # initial_robot_positions = set_straight_line(N, l_0)
    
    # # list of x-y current target positions for each agent (starts at their initial positions)
    # target_pos = [initial_robot_positions[i][:2] for i in range(len(initial_robot_positions))]

    # # a list of all of the agent objects created
    # robot_ids = []

    # # a list of tether objects
    # tether_ids = []

    # # populates the list of robot objects with robot objects
    # for i in range(N):
    #     robot_ids.append(make_robot(radius, initial_robot_positions[i]))

    # # applies friction/damping between robots and the plane
    # for i in range(N):
    #     p.changeDynamics(robot_ids[i], -1, linearDamping=mu)

    # # populates the list of tether objects with tether objects
    # for i in range(N-1):
    #     tether_ids.append(make_tether(initial_robot_positions[i], initial_robot_positions[i+1], l_0, num_segments=1))

    # # anchors all of the tethers to their respective robots
    # for i in range(N-1):
    #     anchor_tether(tether_ids[i], robot_ids[i], robot_ids[i+1])
        
    # runs = 0

    # # print(get_sigma(robot_ids[1], tether_ids[0], tether_ids[1]))

    # # main simulation loop
    # while p.isConnected():
    #     p.getCameraImage(320,200)

    #     if runs%100 == 0:
    #         # calculate tether angle relative to each robot's heading
    #         sigma1 = get_sigma(robot_ids[1], tether_ids[0], tether_ids[1])
    #         # theta2 = get_theta(robot_ids[1], tether_ids[0])

    #         # display results in the GUI
    #         p.addUserDebugText(f"sigma = {sigma1:.2f} deg \n",
    #                             [0, 0.5, 0.5], textColorRGB=[0, 0, 0], lifeTime=1)
        

    #     # # calculate tether length and strain on every step
    #     # l = get_tether_length(tether_ids[0])
    #     # strain = (l - l_0) / l_0

    #     # # calculate tether angle relative to each robot's heading
    #     # sigma1 = get_sigma(robot_ids[1], tether_ids[0], tether_ids[1])
    #     # # theta2 = get_theta(robot_ids[1], tether_ids[0])

    #     # # calculates the direction the robot should move in to achieve the 
    #     # # goal delta
    #     # new_vector = new_position_for_sigma_goal(robot_ids[1], tether_ids[0], tether_ids[1], 270)

    #     # # display results in the GUI
    #     # p.addUserDebugText(f"tether length = {l:.2f} m\n tether strain = {strain:.2f}\n "
    #     #                     f"sigma = {sigma1:.2f} deg \n x_comp = {new_vector[0]:.2f} \n y_comp ={new_vector[1]:.2f} \n",
    #     #                     [0, 0.5, 0.5], textColorRGB=[0, 0, 0], lifeTime=1)
        

    #     if reached_target_position(robot_ids[1], target_pos[1][0], target_pos[1][1], err_pos):
    #         new_pos = calculate_new_position(robot_ids[1], tether_ids[0], tether_ids[1], 90)
    #         target_pos[1] = (new_pos[0], new_pos[1])
    #         move_robot(robot_ids[1], target_pos[1][0], target_pos[1][1], force=60)

    #     if reached_target_position(robot_ids[0], target_pos[0][0], target_pos[0][1], err_pos):
    #         new_pos = new_position_forward_with_strain_1_tether(robot_ids[0], tether_ids[0])
    #         target_pos[0] = (new_pos[0], new_pos[1])
    #         move_robot(robot_ids[0], target_pos[0][0], target_pos[0][1], force=60)

    #     if reached_target_position(robot_ids[2], target_pos[2][0], target_pos[2][1], err_pos):
    #         new_pos = new_position_forward_with_strain_1_tether(robot_ids[2], tether_ids[1])
    #         target_pos[2] = (new_pos[0], new_pos[1])
    #         move_robot(robot_ids[2], target_pos[2][0], target_pos[2][1], force=60)

    #     runs = runs + 1
        
    #     # if runs == 0:
    #     #     new_pos = turn_around(robot_ids[0])
    #     #     target_pos[0] = (new_pos[0], new_pos[1])
    #     #     move_robot(robot_ids[0], target_pos[0][0], target_pos[0][1], force=60)
    #     #     runs = 1
    #     # elif reached_target_position(robot_ids[0], target_pos[0][0], target_pos[0][1], err_pos):
    #     #     new_pos = new_position_forward_with_strain_1_tether(robot_ids[0], tether_ids[0])
    #     #     target_pos[0] = (new_pos[0], new_pos[1])
    #     #     move_robot(robot_ids[0], target_pos[0][0], target_pos[0][1], force=60)


    #     # if reached_target_position(robot_ids[1], target_pos[1][0], target_pos[1][1], err_pos):
    #     #     new_pos = new_position_for_sigma_goal(robot_ids[1], tether_ids[0], tether_ids[1], 150)
    #     #     target_pos[1] = (new_pos[0], new_pos[1])
    #     #     # print(get_sigma(robot_ids[1], tether_ids[0], tether_ids[1]))
    #     #     # print("\n")
    #     #     move_robot(robot_ids[1], target_pos[1][0], target_pos[1][1], force=60)




    #     # if reached_target_position(robot_ids[0], target_pos[0][0], target_pos[0][1], err_pos):
    #     #     new_pos = go_home(robot_ids[0], [-1,1])
    #     #     target_pos[0] = (new_pos[0], new_pos[1])
    #     #     move_robot(robot_ids[0], target_pos[0][0], target_pos[0][1], force=60)

    #     p.stepSimulation()

if __name__ == "__main__":
    main()
