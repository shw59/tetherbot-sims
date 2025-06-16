"""
obstacle.py

This file defines the Obstacle class.
"""

import pybullet as p
import math

class Obstacle:
    label = "obstacle"

    def __init__(self, shape, position, heading, mass, length, width, height, color, mu, fixed):
        """
        Initializes an obstacle with a shape of "hexagon", "cube", or "triangle" at the specified position [x, y] and orientation (in degrees). 
        """
        self.height = height
        urdf_text = ""
        rgba = " ".join(map(str, color))

        if shape == "hexagon":
            urdf_text = f"""<?xml version="1.0"?>
            <robot name="hex_block">
                <link name="hex_block_link">
                    <contact>
                        <lateral_friction value="{mu}"/>
                        <rolling_friction value="{mu}"/>
                    </contact>
                    <visual>
                        <origin xyz="0 0 0" rpy="0 0 0"/>
                        <geometry>
                            <mesh filename="hex.obj" scale="{length} {width} {height}"/>
                        </geometry>
                        <material name="color">
                            <color rgba="{rgba}"/>
                        </material>
                    </visual>

                    <collision>
                        <origin xyz="0 0 0" rpy="0 0 0"/>
                        <geometry>
                            <mesh filename="hex.obj" scale="{length} {width} {height}"/>
                        </geometry>
                    </collision>

                    <inertial>
                        <origin xyz="0 0 0" rpy="0 0 0"/>
                        <mass value="{mass}"/>
                        <inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/>
                    </inertial>
                </link>
            </robot>
            """
        elif shape == "cube":
            urdf_text = f"""<?xml version="1.0" ?>
            <robot name="cube">
                <link name="baseLink">
                    <contact>
                        <lateral_friction value="{mu}"/>
                        <rolling_friction value="{mu}"/>
                    </contact>

                    <inertial>
                        <origin rpy="0 0 0" xyz="0 0 0"/>
                        <mass value="{mass}"/>
                        <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
                    </inertial>

                    <visual>
                        <origin rpy="0 0 0" xyz="0 0 0"/>
                        <geometry>
                            <mesh filename="cube.obj" scale="{length} {width} {height}"/>
                        </geometry>
                        <material name="color">
                            <color rgba="{rgba}"/>
                        </material>
                    </visual>

                    <collision>
                        <origin rpy="0 0 0" xyz="0 0 0"/>
                        <geometry>
                            <box size="{length} {width} {height}"/>
                        </geometry>
                    </collision>
                </link>
            </robot>
            """
        elif shape == "triangle":
            urdf_text = f"""<?xml version="1.0"?>
            <robot name="triangle_block">
                <link name="triangle_block_link">
                    <contact>
                        <lateral_friction value="{mu}"/>
                        <rolling_friction value="{mu}"/>
                    </contact>

                    <visual>
                        <origin xyz="0 0 0" rpy="0 0 0"/>
                        <geometry>
                            <mesh filename="triangle.obj" scale="{length} {width} {height}"/>
                        </geometry>
                        <material name="color">
                            <color rgba="{rgba}"/>
                        </material>
                    </visual>

                    <collision>
                        <origin xyz="0 0 0" rpy="0 0 0"/>
                        <geometry>
                            <mesh filename="triangle.obj" scale="{length} {width} {height}"/>
                        </geometry>
                    </collision>

                    <inertial>
                        <origin xyz="0 0 0" rpy="0 0 0"/>
                        <mass value="{mass}"/>
                        <inertia ixx="0.05" iyy="0.05" izz="0.05" ixy="0" ixz="0" iyz="0"/>
                    </inertial>
                </link>
            </robot>
            """
        
        filename = f"objects/{shape}.urdf"
        open(filename, "w").write(urdf_text)

        position_3d = position + [height / 2]
        self.id = p.loadURDF(filename, position_3d, p.getQuaternionFromEuler([0, 0, math.radians(heading)]))



        if fixed:
            p.createConstraint(self.id, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], position_3d)
        
    def get_pose(self):
        """
        Return the current position and heading orientation of the obstacle as a list [[x, y], heading in degrees].
        """
        curr_pos = p.getBasePositionAndOrientation(self.id)[0][:2]
        curr_orn = math.degrees(p.getEulerFromQuaternion(p.getBasePositionAndOrientation(self.id)[1])[2])

        return [curr_pos, curr_orn]

    def set_pose(self, new_position, heading):
        """
        Set the position and heading orientation of an obstacle object.
        """
        new_position.append(self.height / 2)
        p.resetBasePositionAndOrientation(self.id, new_position, p.getQuaternionFromEuler([0, 0, math.radians(heading)]))