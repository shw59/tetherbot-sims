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
    desired_strain = 0.5
    err_pos = 0.01
    err_delta = 5
    err_strain = 0.05
    angle_weight, strain_weight, gradient_weight, repulsion_weight = [5, 6, 2, 5]

    def __init__(self, position_0, heading_0, radius, mass=1.0, color=(0, 0.5, 1, 1), height=0.01, friction_coeff=2.5):
        """
        Initializes an agent object and its position and id attributes. Heading is angle off of positive x-axis
        """
        self.next_position = position_0[:2]
        self.radius = radius
        self.sensing_radius = radius * 4
        self.desired_tether_angle = None
        self.tethers = [None, None]
        self.cr_sensor_data = []
        self.gradient_sensor_data = None

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

        filename = f"objects/agent.urdf"
        open(filename, "w").write(urdf_text)

        self.id = p.loadURDF(filename, position_0)

        p.resetJointState(self.id, 2, math.radians(heading_0))

        p.changeDynamics(self.id, -1, linearDamping=friction_coeff)
    
    def instantiate_m_tether(self, m_tether):
        """
        Instantiates the negative tether for the agent.
        """
        self.tethers[0] = m_tether

    def instantiate_p_tether(self, p_tether):
        """
        Instantiate the positive tether for the agent.
        """
        self.tethers[1] = p_tether

    def set_desired_tether_angle(self, goal_delta):
        self.desired_tether_angle = goal_delta

    @classmethod
    def set_weights(cls, weight_list):
        Agent.angle_weight, Agent.strain_weight, Agent.gradient_weight, Agent.repulsion_weight = weight_list

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
        n_verts, verts, *_ = p.getMeshData(self.tethers[tether_num].id, -1, flags=p.MESH_DATA_SIMULATION_MESH)

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
        hx, hy = self.get_pose()[1]
        tx, ty = self.get_tether_heading(tether_num)
        theta = math.degrees(math.atan2(hx*ty - hy*tx, hx*tx + hy*ty))

        if theta < 0:
            theta = 360 + theta

        if (tether_num == 1) and (round(theta, 2) == 0):
            theta = 360

        return theta
    
    def get_delta(self):
        """
        Return the angle between two tethers of an agent (in degrees). Returns None if there is no second tether.
        The angle is always positive and inbetween 0-360 degrees.
        """
        if (self.tethers[1] is None) or (self.tethers[0] is None):
            return None
        theta_m = self.get_theta(0)
        theta_p = self.get_theta(1)
        delta = theta_m - theta_p

        if delta < 0:
            delta = 360 + delta

        return delta
    
    def sense_close_range(self, obj_list, sensing_mode=0):
        """
        Three modes of close-range sensing:
        0: agent senses all obstacles, tethers, and agents indistinguishably. Returns list of tuples with unknown type classifications.
        1: agent senses all obstacles, but can distinguish between them. Returns a list of tuples classified as "tether", "agent", or "obstacle".
        2: agent can only sense agents and tethers, but not obstacles. 

        Updates the instance variable sensor_data list with format: (u_r normal vector as numpy array, distance, object type)
        """
        from obstacle import Obstacle
        
        # helper function
        def get_closest_point_distance( obj):
            """
            Returns the closest global coordinate point and distance as (closest_point, dist) from another object to the agent.
            """
            curr_pos = p.getLinkState(self.id, 2)[0][:2]
            closest_points = []
            if obj.label == "tether": # if object is a tether, loop through its vertices and find the closest one to the agent
                _, verts, *_ = p.getMeshData(obj.id, -1, flags=p.MESH_DATA_SIMULATION_MESH)
                closest_point = [float('inf'), float('inf')]
                nearest_dist = float('inf')
                for vert in verts:
                    dist = math.dist(curr_pos, vert[:2])
                    if dist < nearest_dist:
                        nearest_dist = dist
                        closest_point = vert[:2]

                return closest_point, nearest_dist
            elif obj.label == "agent":
                closest_points = p.getClosestPoints(self.id, obj.id, float('inf'), linkIndexA=2, linkIndexB=2)
            elif obj.label == "obstacle":
                closest_points = p.getClosestPoints(self.id, obj.id, float('inf'), linkIndexA=2, linkIndexB=-1)

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
        self.sensor_data = []

        for obj in obj_list:
            closest_point, dist = get_closest_point_distance(obj)
            if obj.id != self.id and dist <= self.sensing_radius:
                if obj.label == "tether" and dist >= self.radius:
                    u_r = utils.normalize(curr_pos - np.array(closest_point))
                elif obj.label != "tether":
                    u_r = utils.normalize(curr_pos - np.array(obj.get_pose()[0]))
                else:
                    continue

                if sensing_mode == 0:
                    self.sensor_data.append((u_r, dist, "unknown"))
                elif sensing_mode == 1:
                    self.sensor_data.append((u_r, dist, obj.label))
                elif sensing_mode == 2:
                    if obj.label != "obstacle":
                        self.sensor_data.append((u_r, dist, "unknown"))
    
    def sense_gradient(self, gradient_source_pos):
        """
        Returns the unit vector from the agent to the gradient source and its linear distance to the gradient source as a tuple.
        """
        if gradient_source_pos is not None:
            curr_pos = self.get_pose()[0]

            u_g = utils.normalize(np.array(gradient_source_pos) - np.array(self.get_pose()[0]))
            dist = math.dist(curr_pos, gradient_source_pos)

            self.gradient_sensor_data = (u_g, dist)
    
    def compute_vector_strain(self, tether_num=0):
        """
        Calculates the vector direction that the agent should move to achieve the goal strain/tautness.
        """
        if self.tethers[tether_num] is None:
            return None
        
        strain_diff = self.tethers[tether_num].get_strain() - Agent.desired_strain

        if strain_diff > Agent.err_strain:
            sign = 1
        elif strain_diff < -Agent.err_strain:
            sign = -1
        else:
            sign = 0

        # The vector from the center of the agent to the point where the tether connects to the agent
        vector_norm = utils.normalize(np.array(self.get_tether_heading(tether_num)))
        v_t = sign * (strain_diff ** 2) * vector_norm

        return v_t
    
    def compute_vector_gradient(self):
        """
        Returns the vector pointing towards the target destination, where the vector's magnitude increases the farther from the
        destination the agent is.
        """
        u_g, distance = self.gradient_sensor_data

        # Changes the weights of the gradient vector depending on how far it is from the source
        if distance >= 30 * self.radius:
            scale = 1
        elif distance >= 10 * self.radius:
            scale = 0.5
        elif distance >= 5 * self.radius:
            scale = 0.01
        else:
            scale = 0
        
        # The vector pointing in the direction of the source
        v_g = scale * u_g

        return v_g
    
    def compute_vector_repulsion(self):
        """
        Returns the repulsion vector given close-range sensor data.
        """
        amplitude = 3 * self.radius
        std_dev = self.radius
        v_r = np.array([0, 0])

        for i in range(len(self.cr_sensor_data)):
            u_r, d, obj_type = self.cr_sensor_data[i]
            v_r = v_r + amplitude * math.exp(-d**2 / (2 * std_dev**2)) * u_r # Gaussian
            # do something here if the obj_type is not "unknown"

        return v_r
    
    def compute_vector_angle(self):
        """
        Computes which direction the agent should move in so that it
        can attempt to reach its desired delta value
        """
        delta = self.get_delta()

        tether_m_heading = self.get_tether_heading(0)
        tether_p_heading = self.get_tether_heading(1)

        normalized_tether_m_heading = utils.normalize(tether_m_heading)
        normalized_tether_p_heading = utils.normalize(tether_p_heading)

        summed_normalized_tether_headings = normalized_tether_m_heading + normalized_tether_p_heading

        magnitude_of_summed_nomralized_headings = utils.magnitude_of_vector(summed_normalized_tether_headings)

        difference = self.desired_tether_angle - delta

        abs_difference = abs(difference)

        # vector = np.array([0,0])

        if (abs_difference > Agent.err_delta):

            if (abs_difference > 10) and (-0.5 <= (normalized_tether_m_heading[0]+normalized_tether_p_heading[0]) <= 0.5) and (-0.5 <= (normalized_tether_m_heading[1]+normalized_tether_p_heading[1]) <= 0.5):
                heading = np.array(self.get_pose()[1])

                vector = np.array(heading)

                normalized_heading = np.array(utils.normalize(heading))
                normalized_t_m = np.array(utils.normalize(tether_m_heading))
                normalized_t_p = np.array(utils.normalize(tether_p_heading))

                # print("heading: " + str(normalized_heading))
                # print("normalized_t_m: " + str(normalized_t_m))
                # print("normalized_t_p: " + str(normalized_t_p))

                if (round(normalized_heading[0]) == round(normalized_t_m[0])) and (round(normalized_heading[1]) == round(normalized_t_m[1])):
                    vector[0] = 20*round(normalized_heading[1])
                    vector[0] = -20*round(normalized_heading[0])
                    # print("enter first")
                    # print("\n")
                elif (round(normalized_heading[0]) == round(normalized_t_p[0])) and (round(normalized_heading[1]) == round(normalized_t_p[1])):
                    vector[0] = 20*round(normalized_heading[1])
                    vector[0] = -20*round(normalized_heading[0])
                    # print("enter second")
                    # print("\n")
                else:

                    vector = np.array(heading)

                    vector[0] = 20*heading[0]
                    vector[1] = 20*heading[1]
                # print("Second if")
                # print("heading: " + str(heading))
                # print("heading x: " + str(heading[0]))
                # print("heading y: " + str(heading[1]))
                # print("vector x: " + str(vector[0]))
                # print("vector y: " + str(vector[1]))

            else:

                if (delta < 180) and (delta > self.desired_tether_angle):
                    sign = -1
                elif (delta < 180) and (delta < self.desired_tether_angle):
                    sign = 1
                elif (delta > 180) and (delta < self.desired_tether_angle):
                    sign = -1
                else:
                    sign = 1

                coefficient = sign * math.sqrt((abs_difference)/(10 * math.pi)) * (magnitude_of_summed_nomralized_headings)

                vector = np.array([coefficient * summed_normalized_tether_headings[0], coefficient * summed_normalized_tether_headings[1]])

            #     print("first else")
            
            # print(vector)
            # print("\n")
            return vector
        else:
            vector = np.array([0,0])

            # print("second else")
            # print(vector)
            # print()
            # print("\n")
            return vector
    
    def compute_next_step(self):
        """
        Calculates and sets the next position the agent should move to based on the resultant weighted vector sum and its current close-range sensor data.
        """
        curr_position = np.array(self.get_pose()[0])
        
        try: # if agent has one tether instantiated
            tether_num = not self.tethers.index(None) # retrieves the list index of the one tether (either 0 or 1)
            v_m_strain = np.array([0, 0]) if tether_num else self.compute_vector_strain(tether_num)
            v_p_strain = self.compute_vector_strain(tether_num) if tether_num else np.array([0, 0])
            v_angle = np.array([0, 0])
        except ValueError: # if agent has two tethers instantiated
            v_m_strain = self.compute_vector_strain(0)
            v_p_strain = self.compute_vector_strain(1)
            v_angle = self.compute_vector_angle()

        if self.gradient_sensor_data is None:
            v_gradient = np.array([0, 0])
        else:
            v_gradient = self.compute_vector_gradient()

        v_repulsion = self.compute_vector_repulsion()

        # print(Agent.strain_weight * (v_m_strain + v_p_strain))
        # print(Agent.gradient_weight * v_gradient)
        # print(Agent.repulsion_weight * v_repulsion)
        # print(Agent.angle_weight * v_angle)
        # print("\n")

        resulting_vector = Agent.strain_weight * (v_m_strain + v_p_strain) + Agent.gradient_weight * v_gradient \
                            + Agent.repulsion_weight * v_repulsion + Agent.angle_weight * v_angle
        
        if utils.magnitude_of_vector(resulting_vector) >= 1:
            self.next_position = curr_position + 0.05*utils.normalize(resulting_vector)
        else:
            self.next_position = curr_position + 0.05*resulting_vector
    
    def move_to(self, force=10):
        """
        Move the agent from its current position to a specified [x, y] target position in the world. The parameter
        target_pos should be a numpy array.
        """
        # amount to move (relative to base position)
        x_move, y_move = self.next_position - np.array(p.getBasePositionAndOrientation(self.id)[0][:2])

        # calculate rotation to face direction of movement
        base_heading = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(self.id)[1])[2] # starting heading
        curr_heading = p.getJointState(self.id, 2)[0]
        
        desired_h_x, desired_h_y = self.next_position - np.array(p.getLinkState(self.id, 2)[0][:2])
        desired_heading = math.atan2(desired_h_y, desired_h_x)

        # rotations must be fed into setJointMotorControl function with respect to the base heading, not current heading
        # use smallest signed angle difference and then add the current heading and base heading
        rotation = base_heading + curr_heading + (desired_heading - curr_heading + math.pi) % (2 * math.pi) - math.pi
        
        joint_indices = [1, 0, 2] # [x-direction, y-direction, rotation/heading]
        p.setJointMotorControlArray(self.id, joint_indices, p.POSITION_CONTROL,
                                    targetPositions=[x_move, y_move, rotation], forces=[force]*3)
        
    def reached_target_position(self):
        """
        Checks to see if the agent's current position is the target position
        """
        position = self.get_pose()[0]

        return ( ( position[0] > self.next_position[0] - Agent.err_pos) and ( position[0] > self.next_position[0] - Agent.err_pos) ) and \
               ( ( position[1] > self.next_position[1] - Agent.err_pos) and ( position[1] > self.next_position[1] - Agent.err_pos) )
        