"""
agent.py

This file defines the agent class for robot objects.
"""

import pybullet as p
import math
from tether import Tether

class Agent:
    def __init__(self, position_0, heading_0, radius, tether, mass=1.0, color=(0, 0.5, 1, 1), height=0.01):
        """
        Initializes an agent object and its position and id attributes.
        """
        self.position = [0, 0]
        self.tether_1 = tether

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

    def curr_pose(self):
        """
        Return the current position and heading of the agent as a list [[x, y], [x_heading, y_heading]].
        """
        agent_pos = p.getLinkState(self.id, 2)[0][:2]
        head_pos = p.getLinkState(self.id, 3)[0][:2]
        heading = [head_pos[i] - agent_pos[i] for i in range(2)]

        return [agent_pos, heading]
    
    def tether_heading(self):
        """
        Return the current heading of the agent's tether with respect to the agent's center.
        """
        n_verts, verts, *_ = p.getMeshData(self.tether.id, -1, flags=p.MESH_DATA_SIMULATION_MESH)

        # get both end vertices of the tether
        p1 = [(verts[0][k] + verts[1][k]) / 2.0 for k in range(2)]
        p2 = [(verts[n_verts - 2][k] + verts[n_verts - 1][k]) / 2.0 for k in range(2)]

        # get the agent's position
        agent_pos = p.getLinkState(self.tether.id, 2)[0][:2]
        
        # check which vertex pair is closer and use the second to next pair to calculate heading
        dist1 = math.dist(agent_pos, p1)
        dist2 = math.dist(agent_pos, p2)
        if dist1 < dist2:
            p1_next = [(verts[1][k] + verts[2][k]) / 2.0 for k in range(2)] # second to end vertex
            heading = [p1_next[i] - agent_pos[i] for i in range(2)]
        else:
            p2_next = [(verts[n_verts - 3][k] + verts[n_verts - 2][k]) / 2.0 for k in range(2)]
            heading = [p2_next[i] - agent_pos[i] for i in range(2)]

        return heading
    
    def theta(self):
        """
        Return the angle between the agent's heading and the heading of its tether (in degrees).
        """
        hx, hy = self.curr_pose()[1]
        tx, ty = self.tether_heading()
        theta = math.atan2(hx*ty - hy*tx, hx*tx + hy*ty)

        return math.degrees(theta) % 360
    
    def move(self, x, y, force=10):
        """
        Move the agent from its current position to a specified [x, y] target position in the world.
        """
        # amount to move (relative to base position)
        x_move = x - p.getBasePositionAndOrientation(self.id)[0][0]
        y_move = y - p.getBasePositionAndOrientation(self.id)[0][1]

        # calculate rotation to face direction of movement
        base_heading = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(self.id)[1])[2] # starting heading
        
        curr_heading = p.getJointState(self.id, 2)[0]

        x_curr_pos, y_curr_pos = p.getLinkState(self.id, 2)[0][:2]
        desired_heading = math.atan2(y - y_curr_pos, x - x_curr_pos)

        # rotations must be fed into setJointMotorControl function with respect to the base heading, not current heading
        # use smallest signed angle difference and then add the current heading and base heading
        rotation = base_heading + curr_heading + (desired_heading - curr_heading + math.pi) % (2 * math.pi) - math.pi
        
        joint_indices = [1, 0, 2] # [x-direction, y-direction, rotation/heading]
        p.setJointMotorControlArray(self.id, joint_indices, p.POSITION_CONTROL,
                                    targetPositions=[x_move, y_move, rotation], forces=[force]*3)