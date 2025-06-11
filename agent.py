"""
agent.py

This file defines the agent class.
"""

import pybullet as p
import math
import numpy as np
from tether import Tether
import utils

class Agent:
    label = "agent"
    desired_strain = 0.1
    gradient_source = None
    angle_weight, strain_weight, gradient_weight, repulsion_weight = [1, 1, 1, 1] # [strain, heading/gradient, collision avoidance, tether angle]

    def __init__(self, goal_delta, position_0, heading_0, radius, mass=1.0, color=(0, 0.5, 1, 1), height=0.01):
        """
        Initializes an agent object and its position and id attributes.
        """
        self.next_position = position_0
        self.radius = radius
        self.sensing_radius = radius * 4
        self.desired_tether_angle = goal_delta

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

    def instantiate_tethers(self, tether_m, tether_p=None):
        """
        Instantiate tether object(s) for the agent.
        """
        self.tethers = [tether_m, tether_p]

    def get_pose(self):
        """
        Return the current position and heading of the agent as a list [[x, y], [x_heading, y_heading]].
        """
        agent_pos = p.getLinkState(self.id, 2)[0][:2]
        head_pos = p.getLinkState(self.id, 3)[0][:2]
        heading = [head_pos[i] - agent_pos[i] for i in range(2)]

        return [agent_pos, heading]
    
    def get_tether_heading(self, tether_num=0):
        """
        Return the current heading of the agent's tether with respect to the agent's center.
        """
        n_verts, verts, *_ = p.getMeshData(self.tether[tether_num].id, -1, flags=p.MESH_DATA_SIMULATION_MESH)

        # get both end vertices of the tether
        p1 = [(verts[0][k] + verts[1][k]) / 2.0 for k in range(2)]
        p2 = [(verts[n_verts - 2][k] + verts[n_verts - 1][k]) / 2.0 for k in range(2)]

        # get the agent's position
        agent_pos = p.getLinkState(self.id, 2)[0][:2]
        
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
    
    def get_theta(self, tether_num=0):
        """
        Return the angle between the agent's heading and the heading of its tether (in degrees).
        """
        hx, hy = self.pose()[1]
        tx, ty = self.tether_heading(tether_num)
        theta = math.atan2(hx*ty - hy*tx, hx*tx + hy*ty)

        return math.degrees(theta) % 360
    
    def get_delta(self):
        """
        Return the angle between two tethers of an agent (in degrees). Returns None if there is no second tether.
        """
        if self.tethers[1] is None:
            return None
        theta_m = self.theta(self.tethers[0])
        theta_p = self.theta(self.tethers[1])
        delta = theta_m - theta_p

        return delta
    
    def robot_sense_close_range(self, obj_list, sensing_mode=0):
        """
        Three modes of close-range sensing:
        0: Robot senses all obstacles, tethers, and robots indistinguishably. Returns list of tuples with unknown type classifications.
        1: Robot senses all obstacles, but can distinguish between them. Returns a list of tuples classified as "tether", "agent", or "obstacle".
        2: Robot can only sense robots and tethers, but not obstacles. 

        The returned list sensor data contains the format (u_r normal vector as numpy array, distance, object type)
        """
        from obstacle import Obstacle
        
        # helper function
        def get_closest_point_distance(self, obj_id, obj_type):
            """
            Returns the closest global coordinate point and distance as (closest_point, dist) from another object to the agent.
            """
            curr_pos = p.getLinkState(self.id, 2)[0][:2]
            closest_points = []
            match obj_type:
                case "tether": # if object is a tether, loop through its vertices and find the closest one to the agent
                    _, verts, *_ = p.getMeshData(obj_id, -1, flags=p.MESH_DATA_SIMULATION_MESH)
                    closest_point = [float('inf'), float('inf')]
                    nearest_dist = float('inf')
                    for vert in verts:
                        dist = math.dist(curr_pos, vert[:2])
                        if dist < nearest_dist:
                            nearest_dist = dist
                            closest_point = vert[:2]

                    return closest_point, nearest_dist
                case "agent":
                    closest_points = p.getClosestPoints(self.id, obj_id, float('inf'), linkIndexA=2, linkIndexB=2)
                case "obstacle":
                    closest_points = p.getClosestPoints(self.id, obj_id, float('inf'), linkIndexA=2, linkIndexB=-1)
            if closest_points: # for obstacles and other agents, loop through the list of closest points and find the closest
                closest_point = closest_points[0][6][:2]
                nearest_dist = math.dist(curr_pos, closest_point)
                for point in closest_points:
                    dist = math.dist(curr_pos, point[6][:2])
                    if dist < nearest_dist:
                        nearest_dist = dist
                        closest_point = point[6][:2]

                return closest_point, nearest_dist
            
            return [float('inf'), float('inf')], float('inf')
        
        curr_pos = np.array(p.getLinkState(self.id, 2)[0][:2])
        sensor_data = []

        for obj in obj_list:
            closest_point, dist = get_closest_point_distance(obj.id, obj.label)
            if obj.id != self.id and dist <= self.sensing_radius:
                if obj.label == "tether" and dist >= self.radius:
                    u_r = utils.normalize_vector(curr_pos - np.array(closest_point))
                elif obj.label != "tether":
                    u_r = utils.normalize_vector(curr_pos - np.array(obj.get_pose()))
                else:
                    continue

                match sensing_mode:
                    case 0:
                        sensor_data.append((u_r, dist, "unknown"))
                    case 1:
                        sensor_data.append((u_r, dist, obj.label))
                    case 2:
                        if obj.label != "obstacle":
                            sensor_data.append((u_r, dist, "unknown"))

        return sensor_data
    
    def compute_vector_strain(self, tether_num=0):
        """
        Calculates the vector direction that the robot should move to achieve the goal strain/tautness.
        """
        strain_diff = self.tether[tether_num].strain() - Agent.desired_strain

        if strain_diff > 0:
            sign = 1
        elif strain_diff < 0:
            sign = -1
        else:
            sign = 0

        # The vector from the center of the robot to the point where the tether connects to the robot
        vector_norm = utils.normalize(np.array(self.tether_heading(tether_num)))
        v_t = sign * (strain_diff ** 2) * vector_norm

        return v_t
    
    def compute_vector_gradient(self):
        """
        Returns the vector pointing towards the target destination, where the vector's magnitude increases the farther from the
        destination the agent is.
        """
        curr_position = self.pose()[0]
        goal_position = Agent.gradient_source
        distance = math.dist(curr_position, goal_position)
        if distance >= 10 * self.length_0:
            scale = 1
        elif distance >= 2 * self.length_0:
            scale = 0.5
        else:
            scale = 0.1
        
        v_g = scale * (1/distance) * (np.array(goal_position) - np.array(curr_position))

        return v_g
    
    def compute_vector_repulsion(self, cr_sensor_data):
        """
        Returns the repulsion vector given close-range sensor data.
        """
        amplitude = 3 * self.radius
        std_dev = self.radius
        v_r = np.array([0, 0])

        for i in range(len(cr_sensor_data)):
            u_r, d, obj_type = cr_sensor_data[i]
            v_r = v_r + amplitude * math.exp(-d**2 / (2 * std_dev**2)) * u_r # Gaussian
            # do something here if the obj_type is not "unknown"

        return v_r
    
    def compute_next_step(self):
        """
        Calculates and sets the next position the agent should move to based on the resultant weighted vector sum.
        """
        curr_position = np.array(self.pose()[0])

        v_strain = self.vector_strain()
        v_gradient = self.vector_gradient()
        v_repulsion = self.vector_repulsion()

        resulting_vector = Agent.strain_weight * v_strain + Agent.gradient_weight * v_gradient + Agent.repulsion_weight * v_repulsion
        
        self.next_position = curr_position + resulting_vector

    def update(self): # feedback control? idk still need to figure out how this will work in context of the main simulation loop
        pass
    
    def move(self, target_pos, force=10):
        """
        Move the agent from its current position to a specified [x, y] target position in the world. The parameter
        target_pos should be a numpy array.
        """
        # amount to move (relative to base position)
        x_move, y_move = target_pos - np.array(p.getBasePositionAndOrientation(self.id)[0][:2])

        # calculate rotation to face direction of movement
        base_heading = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(self.id)[1])[2] # starting heading
        curr_heading = p.getJointState(self.id, 2)[0]
        
        desired_h_x, desired_h_y = target_pos - np.array(p.getLinkState(self.id, 2)[0][:2])
        desired_heading = math.atan2(desired_h_y, desired_h_x)

        # rotations must be fed into setJointMotorControl function with respect to the base heading, not current heading
        # use smallest signed angle difference and then add the current heading and base heading
        rotation = base_heading + curr_heading + (desired_heading - curr_heading + math.pi) % (2 * math.pi) - math.pi
        
        joint_indices = [1, 0, 2] # [x-direction, y-direction, rotation/heading]
        p.setJointMotorControlArray(self.id, joint_indices, p.POSITION_CONTROL,
                                    targetPositions=[x_move, y_move, rotation], forces=[force]*3)
        