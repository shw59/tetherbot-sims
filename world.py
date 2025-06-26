"""
world.py

This file defines the World class, which sets up the simulation environment including all robots, tethers, and obstacles.
"""

import pybullet as p
import pybullet_data
from pybullet_utils import bullet_client as bc
import math
import numpy as np
from agent import Agent
from tether import Tether
from obstacle import Obstacle

GRAVITYZ = -9.81

class World:
    def __init__(self, length, width, time_step=1/240, gui_on=True):
        """
        Initialize the simulation world with attributes dimensions, list of all objects and agents, and gradient source.

        length: Length of the simulation world boundary in meters
        width: Width of the simulation world boundary in meters
        time_step: The simulated time interval between each step calculation
        """
        self.dimensions = [length, width]
        self.obj_list = []
        self.agent_list = []
        self.gradient_source = None

        connection_mode = p.GUI if gui_on else p.DIRECT
        self.id = bc.BulletClient(connection_mode)

        self.id.setAdditionalSearchPath(pybullet_data.getDataPath()) # add pybullet_data to search path

        # set parameters
        self.id.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
        self.id.configureDebugVisualizer(p.COV_ENABLE_GUI,0) # disable side bar windows in the GUI
        self.id.setGravity(0, 0, GRAVITYZ)
        self.id.setTimeStep(time_step)

        # load plane
        self.id.loadURDF("plane.urdf")

        # boundary dimensions
        boundary_height = 0.2
        thickness = 0.01

        # helper function to create boundaries based on specified length and width
        def create_boundary(pos, half_extents):
            """
            Create a rectangular boundary for world objects to stay within.
            """
            collision_shape = self.id.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
            visual_shape = self.id.createVisualShape(p.GEOM_BOX, halfExtents=half_extents,
                                            rgbaColor=[0, 0, 0, 1]) 
            wall = self.id.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=collision_shape,
                baseVisualShapeIndex=visual_shape,
                basePosition=pos
            )
            return wall

        # +x boundary
        create_boundary([length / 2, 0, boundary_height / 2], [thickness, width / 2, boundary_height / 2])
        # -x boundary
        create_boundary([-length / 2, 0, boundary_height / 2], [thickness, width / 2, boundary_height / 2])
        # +y boundary
        create_boundary([0, width / 2, boundary_height / 2], [length / 2, thickness, boundary_height / 2])
        # -y boundary
        create_boundary([0, -width / 2, boundary_height / 2], [length / 2, thickness, boundary_height / 2])
        
    def create_agent(self, position_0, heading_0, radius, goal_delta=None, mass=17, color=(0, 0.5, 1, 1), height=0.01, mu_static=1.25, mu_dynamic=0.9, max_velocity=2, max_velocity_angular=10, drive_power=500):
        """
        Adds an agent to the simulation world and returns its object.
        """
        # print(position_0)
        agent = Agent(position_0, heading_0, radius, mass, color, height, mu_static, mu_dynamic, max_velocity, max_velocity_angular, drive_power)
        agent.world_id = self.id
        agent.set_desired_tether_angle(goal_delta)
        self.obj_list.append(agent)
        self.agent_list.append(agent)

        return agent
    
    def create_and_anchor_tether(self, agent_1, agent_2, tether_length_0, youngs_modulus=600e6, diameter=.0019, num_segments=10, mass=0, mu=0.0):
        """
        Creates and anchors a tether between two specified agent objects, returns the tether object.
        """
        # tether position should be midpoint of two robots
        agent_1_pos = agent_1.get_pose()[0]
        agent_2_pos = agent_2.get_pose()[0]

        tether_x, tether_y = [(agent_1_pos[i] + agent_2_pos[i]) / 2 for i in range(2)]

        tether_pos = [tether_x, tether_y, agent_1.height / 2]

        # solve for the tether's orientation
        diff_x, diff_y = np.array(agent_1_pos) - np.array(agent_2_pos)

        if diff_x >= 0:
            if diff_y >= 0:
                theta = math.pi - math.atan2(diff_x, diff_y)
            else:
                theta = math.atan2(diff_x, -1*diff_y)
        else:
            if diff_y >= 0:
                theta = math.pi + math.atan2(-1*diff_x, diff_y)
            else:
                theta = 2*math.pi - (math.atan2(diff_x, diff_y) - math.pi)

        orientation = p.getQuaternionFromEuler([0, 0, theta]) # [0, 0,-1*theta]

        tether = Tether(tether_pos, tether_length_0, orientation, num_segments, mass, youngs_modulus, diameter, mu) # create the tether object
        tether.world_id = self.id
        self.obj_list.append(tether)
        agent_1.instantiate_p_tether(tether)
        agent_2.instantiate_m_tether(tether)

        # anchor the tether to the two agents
        n_verts, _ = tether.get_verts()

        self.id.createSoftBodyAnchor(tether.id, 0, agent_1.id, 2)
        self.id.createSoftBodyAnchor(tether.id, 1, agent_1.id, 2)
        self.id.createSoftBodyAnchor(tether.id, n_verts-2, agent_2.id, 2)
        self.id.createSoftBodyAnchor(tether.id, n_verts-1, agent_2.id, 2)

        return tether
    
    def create_obstacle(self, shape, position, heading=0, mass=1.0, length=1, width=1, height=1, color=(0, 1, 0, 1), mu_static=1.25, mu_dynamic=0.5, fixed=True):
        """
        Adds an obstacle to the simulation world and returns its object.
        """
        obstacle = Obstacle(shape, position, heading, mass, length, width, height, color, mu_static, mu_dynamic, fixed)
        obstacle.world_id = self.id
        self.obj_list.append(obstacle)

        return obstacle
    
    def set_gradient_source(self, source_pos):
        """
        Set a gradient source in the world for agents to gravitate towards. Must have agents instantiated in the world already.
        """
        self.gradient_source = source_pos

    def display_axis_labels(self):
        """
        Adds x, y, and z axis labels so that it is easier to orient oneself
        """
        self.id.addUserDebugLine([0, 0, 0], [1, 0, 0], lineColorRGB=[0, 0, 1], lineWidth=10, lifeTime=0)
        self.id.addUserDebugLine([0, 0, 0], [0, 1, 0], lineColorRGB=[1, 0, 0], lineWidth=10, lifeTime=0)
        self.id.addUserDebugLine([0, 0, 0], [0, 0, 1], lineColorRGB=[.68, .12, .94], lineWidth=10, lifeTime=0)
        self.id.addUserDebugText("+x", [1, 0, 0], lifeTime=0, textColorRGB=[0, 0, 0])
        self.id.addUserDebugText("+y", [0, 1, 0], lifeTime=0, textColorRGB=[0, 0, 0])
        self.id.addUserDebugText("+z", [0, 0, 1], lifeTime=0, textColorRGB=[0, 0, 0])

    def display_stats(self, labels, values, position=[0, 0, 0.5], color=[0, 0, 0]):
        """
        Display values with a labels in the GUI visualization for clarity. Provide a list of labels and a list of corresponding values.
        """
        text_list = [f"{labels[i]} = {values[i]}" for i in range(len(labels))]
        display_text = ", ".join(text_list)
        self.id.addUserDebugText(display_text, position, textColorRGB=color, lifeTime=1)
