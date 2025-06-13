"""
world.py

This file defines the World class, which sets up the simulation environment including all robots, tethers, and obstacles.
"""

import pybullet as p
import pybullet_data
import math
import numpy as np
from agent import Agent
from tether import Tether
from obstacle import Obstacle

GRAVITYZ = -9.81

class World:
    def __init__(self, length, width, time_step=1/240):
        """
        Initialize the simulation world with length and width boundary dimensions and a time step length for each iteration of the simulation.
        """
        self.obj_list = []
        self.agent_list = []
        self.gradient_source = None

        p.connect(p.GUI) # connect to PyBullet GUI
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) # add pybullet_data to search path

        # set parameters
        p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI,0) # disable side bar windows in the GUI
        p.setGravity(0, 0, GRAVITYZ)
        p.setTimeStep(time_step)

        # load plane
        p.loadURDF("plane.urdf")

        # boundary dimensions
        boundary_height = 0.2
        thickness = 0.01

        # helper function to create boundaries based on specified length and width
        def create_boundary(pos, half_extents):
            """
            Create a rectangular boundary for world objects to stay within.
            """
            collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
            visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents,
                                            rgbaColor=[0, 0, 0, 1]) 
            wall = p.createMultiBody(
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

    def create_agent(self, position_0, heading_0, radius, goal_delta=None, mass=1.0, color=(0, 0.5, 1, 1), height=0.01):
        """
        Adds an agent to the simulation world and returns its object.
        """
        agent = Agent(position_0, heading_0, radius, mass, color, height)
        self.obj_list.append(agent)
        self.agent_list.append(agent)

        return agent
    
    def create_and_anchor_tether(self, agent_1, agent_2, tether_length_0, num_segments=10):
        """
        Creates and anchors a tether between two specified agent objects, returns the tether object.
        """
        # tether position should be midpoint of two robots
        agent_1_pos = agent_1.get_pose()[0]
        agent_2_pos = agent_2.get_pose()[0]

        tether_x, tether_y = [(agent_1_pos[i] + agent_2_pos[i]) / 2 for i in range(2)]

        tether_pos = [tether_x, tether_y, 0]

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

        tether = Tether(tether_pos, tether_length_0, orientation, num_segments) # create the tether object
        self.obj_list.append(tether)
        agent_1.instantiate_p_tether(tether)
        agent_2.instantiate_m_tether(tether)

        # anchor the tether to the two agents
        n_verts, _ = tether.get_verts()

        p.createSoftBodyAnchor(tether.id, 0, agent_1.id, 2)
        p.createSoftBodyAnchor(tether.id, 1, agent_1.id, 2)
        p.createSoftBodyAnchor(tether.id, n_verts-2, agent_2.id, 2)
        p.createSoftBodyAnchor(tether.id, n_verts-1, agent_2.id, 2)

        return tether
    
    def create_obstacle(self, shape, position, heading, mass=1.0, length=1, width=1, height=1, color=(0, 1, 0, 1), fixed=True):
        """
        Adds an obstacle to the simulation world and returns its object.
        """
        obstacle = Obstacle(shape, position, heading, mass, length, width, height, color, fixed)
        self.obj_list.append(obstacle)

        return obstacle
    
    def set_gradient_source(self, source_pos):
        """
        Set a gradient source in the world for agents to gravitate towards. Must have agents instantiated in the world already.
        """
        self.gradient_source = source_pos
