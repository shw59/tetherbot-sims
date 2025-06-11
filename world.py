"""
world.py

This file sets up the environment to run the simulation, 
including all robots, tethers, and obstacles.
"""

import pybullet as p
import pybullet_data

GRAVITYZ = -9.81

class World:
    obj_list = []

    def __init__(self, length, width):
        """
        Initialize the simulation world with length and width boundary dimensions that will contain all objects.
        """
        p.connect(p.GUI) # connect to PyBullet GUI
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) # add pybullet_data to search path

        # set parameters
        p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI,0) # disable side bar windows in the GUI
        p.setGravity(0, 0, GRAVITYZ)
        p.setTimeStep(1./240.)

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
            wall = p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=collision_shape,
                basePosition=pos
            )
            return wall

        # +x boundary
        create_boundary([length / 2, 0, boundary_height / 2], [thickness, length / 2, boundary_height / 2])
        # -x boundary
        create_boundary([-length / 2, 0, boundary_height / 2], [thickness, length / 2, boundary_height / 2])
        # +y boundary
        create_boundary([0, length / 2, boundary_height / 2], [length / 2, thickness, boundary_height / 2])
        # -y boundary
        create_boundary([0, -length / 2, boundary_height / 2], [length / 2, thickness, boundary_height / 2])

    def create_agent()