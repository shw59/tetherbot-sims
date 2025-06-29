"""
obstacle.py

This file defines the Obstacle class.
"""

import pybullet as p
import math
import utils
import os

class Obstacle:
    label = "obstacle"
    joint_indices = [1, 0, 2] # [x, y, rotation]

    def __init__(self, shape, position, heading, mass, length, width, height, color, mu_static, mu_dynamic, fixed):
        """
        Initializes an obstacle with attributes height and id. 

        shape: One of "hexagon", "cube", or "triangle" strings
        position: Initial position of the obstacle in format [x, y, z]
        heading: Orientation of the obstacle in degrees, where the scale ranges from 0 to 360 degrees beginning from the +x axis
        mass: Mass of the obstacle in kg
        length: Length of the obstacle in meters
        width: Width of the obstacle in meters
        height: height of the obstacle in meters
        color: Color of the obstacle in RGBA format (r, g, b, a) where each color component ranges from 0 to 1
        mu_static: Obstacle's static coefficient of friction (only applicable if not movable)
        mu_dynamic: Obstacle's dynamic coefficient of friction (only applicable if not movable)
        fixed: True if the obstacle is fixed and False if the obstacle is movable in the 2D plane
        """
        self.world_id = p
        self.height = height
        self.collected = False # only used for movable obstacles by a main loop to flag objects collected by an agent formation

        urdf_text = ""
        rgba = " ".join(map(str, color))

        if shape == "hexagon":
            urdf_text = f"""<?xml version="1.0"?>
            <robot name="hex_block">
                <link name="world">
                    <inertial>
                        <mass value="0"/>
                        <origin rpy="0 0 0" xyz="0 0 0"/>
                        <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
                    </inertial>
                </link>

                <joint name="y_to_world" type="prismatic">
                    <parent link="world"/>
                    <child link="y_prismatic"/>
                    <axis xyz="0 1 0"/>
                    <limit effort="0.0" lower="1" upper="-1" velocity="1000.0"/>
                    <origin rpy="0 0 0" xyz="0 0 0"/>
                </joint>

                <link name="y_prismatic">
                    <inertial>
                        <mass value="0.01"/>
                        <inertia ixx="0.2125" ixy="-0.005" ixz="0.0225" iyy="0.205" iyz="0.045" izz="0.0125"/>
                        <origin rpy="0 0 0" xyz="0 0 0"/>
                    </inertial>
                </link>

                <joint name="x_to_y" type="prismatic">
                    <parent link="y_prismatic"/>
                    <child link="x_prismatic"/>
                    <axis xyz="1 0 0"/>
                    <limit effort="0.0" lower="1" upper="-1" velocity="1000.0"/>
                    <origin rpy="0 0 0" xyz="0 0 0"/>
                </joint>

                <link name="x_prismatic">
                    <inertial>
                        <mass value="0.01"/>
                        <inertia ixx="0.2125" ixy="-0.005" ixz="0.0225" iyy="0.205" iyz="0.045" izz="0.0125"/>
                        <origin rpy="0 0 0" xyz="0 0 0"/>
                    </inertial>
                </link>

                <joint name="hex_to_x" type="continuous">
                    <parent link="x_prismatic"/>
                    <child link="hex_block_link"/>
                    <axis xyz="0 0 1"/>
                    <origin rpy="0 0 0" xyz="0 0 0"/>
                </joint>

                <link name="hex_block_link">
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
        elif shape == "cylinder":
            urdf_text = f"""<?xml version="1.0"?>
            <robot name="cyl_block">
                <link name="world">
                    <inertial>
                        <mass value="0"/>
                        <origin rpy="0 0 0" xyz="0 0 0"/>
                        <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
                    </inertial>
                </link>

                <joint name="y_to_world" type="prismatic">
                    <parent link="world"/>
                    <child link="y_prismatic"/>
                    <axis xyz="0 1 0"/>
                    <limit effort="0.0" lower="1" upper="-1" velocity="1000.0"/>
                    <origin rpy="0 0 0" xyz="0 0 0"/>
                </joint>

                <link name="y_prismatic">
                    <inertial>
                        <mass value="0.01"/>
                        <inertia ixx="0.2125" ixy="-0.005" ixz="0.0225" iyy="0.205" iyz="0.045" izz="0.0125"/>
                        <origin rpy="0 0 0" xyz="0 0 0"/>
                    </inertial>
                </link>

                <joint name="x_to_y" type="prismatic">
                    <parent link="y_prismatic"/>
                    <child link="x_prismatic"/>
                    <axis xyz="1 0 0"/>
                    <limit effort="0.0" lower="1" upper="-1" velocity="1000.0"/>
                    <origin rpy="0 0 0" xyz="0 0 0"/>
                </joint>

                <link name="x_prismatic">
                    <inertial>
                        <mass value="0.01"/>
                        <inertia ixx="0.2125" ixy="-0.005" ixz="0.0225" iyy="0.205" iyz="0.045" izz="0.0125"/>
                        <origin rpy="0 0 0" xyz="0 0 0"/>
                    </inertial>
                </link>

                <joint name="cyl_to_x" type="continuous">
                    <parent link="x_prismatic"/>
                    <child link="cyl_block_link"/>
                    <axis xyz="0 0 1"/>
                    <origin rpy="0 0 0" xyz="0 0 0"/>
                </joint>

                <link name="cyl_block_link">
                    <visual>
                        <origin xyz="0 0 {height/2}" rpy="0 0 0"/>
                        <geometry>
                            <mesh filename="objects/cylinder.obj" scale="{length} {width} {height}"/>
                        </geometry>
                        <material name="color">
                            <color rgba="{rgba}"/>
                        </material>
                    </visual>

                    <collision>
                        <origin xyz="0 0 {height/2}" rpy="0 0 0"/>
                        <geometry>
                            <mesh filename="objects/cylinder.obj" scale="{length} {width} {height}"/>
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
                <link name="world">
                    <inertial>
                        <mass value="0"/>
                        <origin rpy="0 0 0" xyz="0 0 0"/>
                        <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
                    </inertial>
                </link>

                <joint name="y_to_world" type="prismatic">
                    <parent link="world"/>
                    <child link="y_prismatic"/>
                    <axis xyz="0 1 0"/>
                    <limit effort="0.0" lower="1" upper="-1" velocity="1000.0"/>
                    <origin rpy="0 0 0" xyz="0 0 0"/>
                </joint>

                <link name="y_prismatic">
                    <inertial>
                        <mass value="0.01"/>
                        <inertia ixx="0.2125" ixy="-0.005" ixz="0.0225" iyy="0.205" iyz="0.045" izz="0.0125"/>
                        <origin rpy="0 0 0" xyz="0 0 0"/>
                    </inertial>
                </link>

                <joint name="x_to_y" type="prismatic">
                    <parent link="y_prismatic"/>
                    <child link="x_prismatic"/>
                    <axis xyz="1 0 0"/>
                    <limit effort="0.0" lower="1" upper="-1" velocity="1000.0"/>
                    <origin rpy="0 0 0" xyz="0 0 0"/>
                </joint>

                <link name="x_prismatic">
                    <inertial>
                        <mass value="0.01"/>
                        <inertia ixx="0.2125" ixy="-0.005" ixz="0.0225" iyy="0.205" iyz="0.045" izz="0.0125"/>
                        <origin rpy="0 0 0" xyz="0 0 0"/>
                    </inertial>
                </link>

                <joint name="cube_to_x" type="continuous">
                    <parent link="x_prismatic"/>
                    <child link="baseLink"/>
                    <axis xyz="0 0 1"/>
                    <origin rpy="0 0 0" xyz="0 0 0"/>
                </joint>

                <link name="baseLink">
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
                <link name="world">
                    <inertial>
                        <mass value="0"/>
                        <origin rpy="0 0 0" xyz="0 0 0"/>
                        <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
                    </inertial>
                </link>

                <joint name="y_to_world" type="prismatic">
                    <parent link="world"/>
                    <child link="y_prismatic"/>
                    <axis xyz="0 1 0"/>
                    <limit effort="0.0" lower="1" upper="-1" velocity="1000.0"/>
                    <origin rpy="0 0 0" xyz="0 0 0"/>
                </joint>

                <link name="y_prismatic">
                    <inertial>
                        <mass value="0.01"/>
                        <inertia ixx="0.2125" ixy="-0.005" ixz="0.0225" iyy="0.205" iyz="0.045" izz="0.0125"/>
                        <origin rpy="0 0 0" xyz="0 0 0"/>
                    </inertial>
                </link>

                <joint name="x_to_y" type="prismatic">
                    <parent link="y_prismatic"/>
                    <child link="x_prismatic"/>
                    <axis xyz="1 0 0"/>
                    <limit effort="0.0" lower="1" upper="-1" velocity="1000.0"/>
                    <origin rpy="0 0 0" xyz="0 0 0"/>
                </joint>

                <link name="x_prismatic">
                    <inertial>
                        <mass value="0.01"/>
                        <inertia ixx="0.2125" ixy="-0.005" ixz="0.0225" iyy="0.205" iyz="0.045" izz="0.0125"/>
                        <origin rpy="0 0 0" xyz="0 0 0"/>
                    </inertial>
                </link>

                <joint name="triangle_to_x" type="continuous">
                    <parent link="x_prismatic"/>
                    <child link="triangle_block_link"/>
                    <axis xyz="0 0 1"/>
                    <origin rpy="0 0 0" xyz="0 0 0"/>
                </joint>

                <link name="triangle_block_link">
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

        position_3d = position + [0]
        self.id = self.world_id.loadURDF(os.path.abspath(filename), position_3d, p.getQuaternionFromEuler([0, 0, math.radians(heading)]))

        if fixed:
            self.world_id.createConstraint(self.id, 2, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], position_3d)
        else:
            # set dynamic friction coefficient
            for i in range(3):
                self.world_id.changeDynamics(self.id, i, jointDamping=mu_dynamic)

            # set static friction coefficient
            force_friction = utils.normal_force(mass) * mu_static
            self.world_id.setJointMotorControlArray(self.id, Obstacle.joint_indices, controlMode=p.VELOCITY_CONTROL, forces=[force_friction]*3)
        
    def get_pose(self):
        """
        Return the current position and heading orientation of the obstacle as a list [[x, y], heading in degrees].
        """
        curr_pos = self.world_id.getLinkState(self.id, 2)[0][:2]
        curr_orn = math.degrees(p.getEulerFromQuaternion(self.world_id.getLinkState(self.id, 2)[1])[2])

        return [curr_pos, curr_orn]

    def set_pose(self, new_position, heading):
        """
        Set the position and heading orientation of an obstacle object.

        new_position: Newly specified [x, y, z] position of an obstacle object
        heading: Newly specified heading of an obstacle object in degrees ranging from 0 to 360 starting from the +x axis
        """
        new_position.append(self.height / 2)
        self.world_id.resetBasePositionAndOrientation(self.id, new_position, p.getQuaternionFromEuler([0, 0, math.radians(heading)]))