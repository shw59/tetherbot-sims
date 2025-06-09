"""
agent.py

This file defines the agent class for robot objects.
"""

import pybullet as p
import math

class Agent:
    def __init__(self, position_0, heading_0, radius, mass=1.0, color=(0, 0.5, 1, 1), height=0.01):
        """
        Initializes an agent object and its position and id attributes.
        """
        self.position = position_0
        self.heading = heading_0

        # inertia of a solid cylinder about its own center
        ixx = iyy = (1/12) * mass * (3 * radius**2 + height**2)
        izz = 0.5 * mass * radius**2

        rgba = " ".join(map(str, color))

        # set position of heading-indicator block
        block_origin_x = radius / 2
        block_origin_z = height / 2

        urdf_text = f"""<?xml version="1.0"?>
        <robot name="disk">
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

            <joint name="base_to_x" type="continuous">
                <parent link="x_prismatic"/>
                <child link="base_link"/>
                <axis xyz="0 0 1"/>
                <origin rpy="0 0 0" xyz="0 0 0"/>
            </joint>

            <link name="base_link">
                <visual>
                    <origin xyz="0 0 {height/2}" rpy="0 0 0"/>
                    <geometry>
                        <cylinder length="{height}" radius="{radius}"/>
                    </geometry>
                    <material name="agent_color">
                        <color rgba="{rgba}"/>
                    </material>
                </visual>

                <collision>
                    <origin xyz="0 0 {height/2}" rpy="0 0 0"/>
                    <geometry>
                        <cylinder length="{height}" radius="{radius}"/>
                    </geometry>
                </collision>

                <inertial>
                    <origin xyz="0 0 0" rpy="0 0 0"/>
                    <mass value="{mass}"/>
                    <inertia ixx="{ixx:.6f}" ixy="0" ixz="0" iyy="{iyy:.6f}" iyz="0" izz="{izz:.6f}"/>
                </inertial>
            </link>

            <joint name= "block_to_base" type="fixed">
                <parent link="base_link"/>
                <child link="heading_block"/>
                <origin xyz="{block_origin_x} 0 {block_origin_z}" rpy="0 0 0"/>
            </joint>

            <link name="heading_block">
                <visual>
                    <origin xyz="{block_origin_x} 0 {block_origin_z}" rpy="0 0 0"/>
                    <geometry>
                        <box size="{radius} 0.01 0.01"/>
                    </geometry>
                    <material name="block_color"><color rgba="0 0 1 1"/></material>
                </visual>
            </link>
        </robot>
        """

        robot_blue_filename = f"agent.urdf"
        open(robot_blue_filename, "w").write(urdf_text)

        self.id = p.loadURDF(robot_blue_filename, position_0)

        p.resetJointState(self.id, 2, math.radians(heading_0))