"""
agent.py

This file defines the agent class.
"""

import pybullet as p
import math
import numpy as np
import utils
import os

class Agent:
    label = "agent" # used for the identification of an agent object when sensing
    joint_indices = [1, 0, 2] # [x-direction, y-direction, rotation/heading]
    desired_strain = 0.15 # how much strain the agents will attempt to maintain in their tethers, if they have any
    err_pos = 0.5
    err_delta = 7 # The allowable error in the accuracy of the goal angle between an agent's two tether
    err_strain = 0.05 # The allowable error in the accuracy of the goal strain that the agent's want to maintain
    err_heading = 10
    err_velocity = .1
    angle_weight, strain_weight, gradient_weight, repulsion_weight = [5, 6, 2, 5] # The different weightings for the resultant vector, see compute_next_step

    @classmethod
    def set_weights(cls, weight_list):
        """
        Sets the values of the weights used to compute the resultant vector, which the agent uses to determine where to move
        """
        Agent.angle_weight, Agent.strain_weight, Agent.gradient_weight, Agent.repulsion_weight = weight_list

    # Initializes an agent object
    def __init__(self, position_0, heading_0, radius, mass, color, height, mu_static, mu_dynamic, max_velocity, max_velocity_angular, drive_power):
        """
        Initializes an agent object and its position and id attributes.

        position_0: a three dimenstional vector in the form [x, y, z] (z is the offset from the plane where a z of 0 means the agent is resting exactly on top of the plane)
        heading_0: the direction the heading will face, a float in degrees whose angle is measured from the +x-axis
        radius: the radius of the cylindrical body of the agent, a float
        mass: the mass of the agent, a float
        color: the color of the agent, (r, g, b, 1), where r/g/b are either 0 or 1 to create different colors
        height: hieght of the cylindrical body of the agent, a float
        mu_static: the coefficient of static friction, a float
        mu_dynamic: the coefficient of dynamic friction, a float
        max_velocity: the maximum velocity that the agent can move in m/s, a float
        """
        self.world_id = p
        self.height = height # height of the agent
        self.next_position = position_0[:2] # gets the (x,y) of the position
        self.max_velocity = max_velocity
        self.max_velocity_angular = max_velocity_angular
        self.max_force = drive_power / max_velocity
        self.radius = radius # sets the radius of the agent
        self.sensing_radius = radius * 1.5 # sets the sensing radius of the agent
        self.desired_tether_angle = None # initializes the desired tether angle to be None
        self.tethers = [None, None] # initializes no attached tethers to the agent
        self.cr_sensor_data = [] # initializes an empty list of sensor data
        self.gradient_sensor_data = None # initializes the gradient sensor data to none
        self.failed = False # True if agent has failed and stopped working

        # inertia of a solid cylinder about its own center
        ixx = iyy = (1/12) * mass * (3 * radius**2 + height**2)
        izz = 0.5 * mass * radius**2

        rgba = " ".join(map(str, color))

        # set position of heading-indicator block
        block_origin_x = radius / 2
        block_origin_z = height / 2

        # the construction of the agent
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

        self.id = self.world_id.loadURDF(os.path.abspath(filename), position_0)

        # set initial heading
        self.world_id.resetJointState(self.id, 2, math.radians(heading_0))

        # set dynamic friction coefficient
        for i in range(3):
            self.world_id.changeDynamics(self.id, i, jointDamping=mu_dynamic)

        # set static friction coefficient
        self.force_friction_static = utils.normal_force(mass) * mu_static
        self.world_id.setJointMotorControlArray(self.id, Agent.joint_indices, controlMode=p.VELOCITY_CONTROL, targetVelocities=[0, 0, 0], forces=[self.force_friction_static]*3)
    
    def instantiate_m_tether(self, m_tether):
        """
        Instantiates the negative tether for the agent.
        m_tehter: a tether object
        """
        self.tethers[0] = m_tether

    def instantiate_p_tether(self, p_tether):
        """
        Instantiate the positive tether for the agent.
        p_tether: a tether object
        """
        self.tethers[1] = p_tether

    def set_desired_tether_angle(self, goal_delta):
        """
        Sets the value of the desired tether angle.
        goal_delta: a float from 0 to 360, in degrees, measured from the m_tether to
                    the p_tether from the heading. Should not be negative. If it is 
                    negative, add 360 to it.
        """
        self.desired_tether_angle = goal_delta

    def get_pose(self):
        """
        Returns the current position, vector heading, and angle heading of the agent as a list [[x, y], [x_heading, y_heading], angle (degrees)].
        The [x_heading, y_heading] is a vector that points in the direction of the heading of the agent.

        Note: the angle heading is unbounded and goes negative for CW and positive for CCW, and based on PyBullet's own angle calculation of a continuous joint
        """
        agent_pos = self.world_id.getLinkState(self.id, 2)[0][:2]
        head_pos = self.world_id.getLinkState(self.id, 3)[0][:2]
        heading = [head_pos[i] - agent_pos[i] for i in range(2)]
        angle = math.degrees(self.world_id.getJointState(self.id, 2)[0])

        return [agent_pos, heading, angle]
    
    def get_velocity(self):
        """
        Returns the current velocity of the agent
        """
        x_velocity = self.world_id.getJointState(self.id, 1)[1]
        y_velocity = self.world_id.getJointState(self.id, 0)[1]
        total_velocity = math.sqrt(x_velocity**2 + y_velocity**2)

        return total_velocity
    
    def get_tether_heading(self, tether_num=0):
        """
        Returns the current heading of the agent's tether with respect to the agent's center.
        tether_num: either 0 or 1, representing the m_tether or the p_tether, respectively
        """
        n_verts, verts, *_ = self.world_id.getMeshData(self.tethers[tether_num].id, -1, flags=p.MESH_DATA_SIMULATION_MESH)

        # get both end vertices of the tether
        p1 = [(verts[0][k] + verts[1][k]) / 2.0 for k in range(2)]
        p2 = [(verts[n_verts - 2][k] + verts[n_verts - 1][k]) / 2.0 for k in range(2)]

        # get the agent's position
        agent_pos = self.world_id.getLinkState(self.id, 2)[0][:2]
        
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
        Returns the angle between the agent's heading and the heading of one of its tether (in degrees).
        tether_num: either 0 or 1, representing the m_tether or the p_tether, respectively
        """
        hx, hy = self.get_pose()[1]
        tx, ty = self.get_tether_heading(tether_num)
        theta = (math.degrees(math.atan2(hx*ty - hy*tx, hx*tx + hy*ty))) % 360

        if (tether_num == 1) and (round(theta, 2) == 0):
            theta = 360

        return theta
    
    def get_delta(self):
        """
        Returns the angle between two tethers of an agent (in degrees). 
        Returns None if there is no second tether. The angle is always positive and in between 0-360 degrees.
        """
        if (self.tethers[1] is None) or (self.tethers[0] is None):
            return None
        theta_m = self.get_theta(0)
        theta_p = self.get_theta(1)
        delta = (theta_m - theta_p) % 360

        return delta
    
    def is_tether_slack(self):
        """
        Checks whether any one of the agent's tethers are slack and returns true if it is.
        """
        for tether in self.tethers:
            if tether is not None and tether.get_strain() < 0:
                self.tether_slack = True
                return True
            
        return False
    
    def sense_close_range(self, obj_list, sensing_mode=0):
        """
        Updates the instance variable sensor_data list with 
        format (u_r normal vector as numpy array, distance, object type)

        sensing_mode: can be 0, 1, or 2, with the following effects:
            0: agent senses all obstacles, tethers, and agents indistinguishably. Returns 
            a list of tuples with unknown type classifications.
            1: agent senses all obstacles, but can distinguish between them. Returns a 
            list of tuples classified as "tether", "agent", or "obstacle".
            2: agent can only sense agents and tethers, but not obstacles. 
        """
        from obstacle import Obstacle
        
        # helper function
        def get_closest_point_distance(obj):
            """
            Returns the closest global coordinate point and distance 
            as (closest_point, dist) from another object to the agent.
            """
            curr_pos = self.get_pose()[0]
            closest_points = []
            if obj.label == "tether":
                verts = obj.get_verts()[1]
                closest_point = [float('inf'), float('inf')]
                nearest_dist = float('inf')
                for vert in verts:
                    dist = math.dist(curr_pos, vert[:2])
                    if dist < nearest_dist:
                        nearest_dist = dist
                        closest_point = vert[:2]

                return closest_point, nearest_dist
            elif (obj.label == "agent" or obj.label == "obstacle"):
                closest_points = self.world_id.getClosestPoints(self.id, obj.id, float('inf'), linkIndexA=2, linkIndexB=2)
            # match obj.label:
            #     case "tether": # if object is a tether, loop through its vertices and find the closest one to the agent
            #         verts = obj.get_verts()[1]
            #         closest_point = [float('inf'), float('inf')]
            #         nearest_dist = float('inf')
            #         for vert in verts:
            #             dist = math.dist(curr_pos, vert[:2])
            #             if dist < nearest_dist:
            #                 nearest_dist = dist
            #                 closest_point = vert[:2]

            #         return closest_point, nearest_dist
            #     case "agent" | "obstacle":
            #         closest_points = self.world_id.getClosestPoints(self.id, obj.id, float('inf'), linkIndexA=2, linkIndexB=2)

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
        
        curr_pos = np.array(self.get_pose()[0])
        self.cr_sensor_data = []

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
                    self.cr_sensor_data.append((u_r, dist, "unknown"))
                elif sensing_mode == 1:
                    self.cr_sensor_data.append((u_r, dist, obj.label))
                elif sensing_mode == 2:
                    if obj.label != "obstacle":
                        self.cr_sensor_data.append((u_r, dist, "unknown"))
                
                # match sensing_mode:
                #     case 0:
                #         self.cr_sensor_data.append((u_r, dist, "unknown"))
                #     case 1:
                #         self.cr_sensor_data.append((u_r, dist, obj.label))
                #     case 2:
                #         if obj.label != "obstacle":
                #             self.cr_sensor_data.append((u_r, dist, "unknown"))
    
    def sense_gradient(self, gradient_source_pos):
        """
        Sets the gradient_sensor_data parameter to a tuple with the unit vector from the agent 
        to the gradient source as the first entry and its linear distance to the gradient 
        source as its second entry.
        gradient_source_pos: the [x,y] position of the gradient source
        """
        if gradient_source_pos is not None:
            curr_pos = self.get_pose()[0]

            u_g = utils.normalize(np.array(gradient_source_pos) - np.array(self.get_pose()[0]))
            dist = math.dist(curr_pos, gradient_source_pos)

            self.gradient_sensor_data = (u_g, dist)
    
    def compute_vector_strain(self, tether_num=0):
        """
        Calculates and returns the vector direction that the agent should 
        move to achieve the goal strain/tautness.
        tether_num: either 0 or 1, representing the m_tether or the p_tether, respectively
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
        v_t = sign * 8 * (strain_diff ** 2) * vector_norm

        return v_t
    
    def compute_vector_gradient(self):
        """
        Returns the vector pointing towards the target destination, where the 
        vector's magnitude increases the farther from the destination the agent is.
        """
        v_g = np.array([0, 0])

        if self.gradient_sensor_data is not None:
            u_g, distance = self.gradient_sensor_data

            if distance > Agent.err_pos:
                scale = len(self.cr_sensor_data) < 3

                v_g = 0.36 * scale * (1 - math.exp(-distance**2 / (self.radius * 30))) * u_g

        return v_g
    
    def compute_vector_repulsion(self):
        """
        Returns the repulsion vector given close-range sensor data.
        """
        v_r = np.array([0, 0])

        for i in range(len(self.cr_sensor_data)):
            u_r, d, obj_type = self.cr_sensor_data[i]
            v_r = v_r + 0.54 * math.exp(-d**2 / self.sensing_radius) * u_r # Gaussian
            # can do something here if the obj_type is not "unknown"

        return v_r
    
    def compute_vector_angle(self):
        """
        Returns which direction the agent should move in so that it
        can attempt to reach its desired delta value
        """
        vector = np.array([0, 0])

        if self.desired_tether_angle is not None:
            delta = self.get_delta()

            tether_m_heading = np.array(self.get_tether_heading(0))
            tether_p_heading = np.array(self.get_tether_heading(1))

            normalized_tether_m_heading = utils.normalize(tether_m_heading)
            normalized_tether_p_heading = utils.normalize(tether_p_heading)

            summed_normalized_tether_headings = normalized_tether_m_heading + normalized_tether_p_heading

            magnitude_of_summed_nomralized_headings = utils.magnitude_of_vector(summed_normalized_tether_headings)

            difference = self.desired_tether_angle - delta

            abs_difference = abs(difference)

            if (abs_difference > Agent.err_delta):

                if (abs_difference > 10) and (-0.5 <= (normalized_tether_m_heading[0]+normalized_tether_p_heading[0]) <= 0.5) and (-0.5 <= (normalized_tether_m_heading[1]+normalized_tether_p_heading[1]) <= 0.5):
                    heading = np.array(self.get_pose()[1])

                    vector = np.array(heading)

                    normalized_heading = np.array(utils.normalize(heading))
                    normalized_t_m = np.array(utils.normalize(tether_m_heading))
                    normalized_t_p = np.array(utils.normalize(tether_p_heading))

                    if (round(normalized_heading[0]) == round(normalized_t_m[0])) and (round(normalized_heading[1]) == round(normalized_t_m[1])):
                        vector[0] = 20*round(normalized_heading[1])
                        vector[1] = -20*round(normalized_heading[0])

                    elif (round(normalized_heading[0]) == round(normalized_t_p[0])) and (round(normalized_heading[1]) == round(normalized_t_p[1])):
                        vector[0] = -20*round(normalized_heading[1])
                        vector[1] = 20*round(normalized_heading[0])
               
                    else:
                        vector = np.array(heading)
                        vector = 20 * np.array(heading)

                else:
                    if (delta < 180) and (delta > self.desired_tether_angle):
                        sign = -1
                    elif (delta < 180) and (delta < self.desired_tether_angle):
                        sign = 1
                    elif (delta > 180) and (delta < self.desired_tether_angle):
                        sign = -1
                    else:
                        sign = 1

                    coefficient = sign * math.sqrt((math.radians(abs_difference))/(2 * math.pi)) * (magnitude_of_summed_nomralized_headings)

                    vector = 0.36 * coefficient * summed_normalized_tether_headings

        return vector
    
    def set_next_step(self):
        """
        Calculates and sets the next position the agent should move to based on 
        the resultant weighted vector sum and its current close-range sensor data.
        The function then calls move_to so that the robot begins to move in that direction.
        """
        self.stop_move()
        if not self.failed:
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

            v_gradient = self.compute_vector_gradient()

            v_repulsion = self.compute_vector_repulsion()

            resulting_vector = Agent.strain_weight * (v_m_strain + v_p_strain) + Agent.gradient_weight * v_gradient \
                                + Agent.repulsion_weight * v_repulsion + Agent.angle_weight * v_angle
            
            # print(f"v_m_strain: {v_m_strain}", f"magnitude: {utils.magnitude_of_vector(v_m_strain)}")
            # print(f"v_p_strain: {v_p_strain}", f"magnitude: {utils.magnitude_of_vector(v_p_strain)}")
            # print(f"v_angle: {v_angle}", f"magnitude: {utils.magnitude_of_vector(v_angle)}")
            # print(f"v_gradient: {v_gradient}", f"magnitude: {utils.magnitude_of_vector(v_gradient)}")
            # print(f"v_repulsion: {v_repulsion}", f"magnitude: {utils.magnitude_of_vector(v_repulsion)}")
            # print(f"resulting_vector: {resulting_vector}", f"magnitude: {utils.magnitude_of_vector(resulting_vector)}\n")
            
            if utils.magnitude_of_vector(resulting_vector) > 0.25:
                self.next_position = curr_position + 0.25 * utils.normalize(resulting_vector)
            else:
                self.next_position = curr_position + resulting_vector

            self.move_to(self.max_force)
            
    def stop_move(self):
        """
        Stop the agent's current motion and check that it is stopped.
        """
        self.world_id.setJointMotorControlArray(self.id, Agent.joint_indices, p.VELOCITY_CONTROL, targetVelocities=[0, 0, 0], forces=[self.force_friction_static]*3)
        while self.world_id.getJointState(self.id, 2)[1] > Agent.err_velocity or self.world_id.getJointState(self.id, 2)[1] < -Agent.err_velocity:
            self.world_id.getCameraImage(320,200)
            self.world_id.stepSimulation()

    def move_to(self, force=float('inf')):
        """
        Moves the agent from its current position to its next_position.
        """
        # amount to move (relative to base position)
        x_move, y_move = self.next_position - np.array(self.world_id.getBasePositionAndOrientation(self.id)[0][:2])

        # calculate rotation to face direction of movement
        base_heading = p.getEulerFromQuaternion(self.world_id.getBasePositionAndOrientation(self.id)[1])[2] # starting heading
        curr_heading = math.radians(self.get_pose()[2])
        
        desired_h_x, desired_h_y = self.next_position - np.array(self.get_pose()[0])
        desired_heading = math.atan2(desired_h_y, desired_h_x)

        # rotations must be fed into setJointMotorControl function with respect to the base heading, not current heading
        # use smallest signed angle difference and then add the current heading and base heading
        rotation = base_heading + curr_heading + (desired_heading - curr_heading + math.pi) % (2 * math.pi) - math.pi

        # set the robot to rotate to the desired heading and check that it reaches that heading before moving forward
        self.world_id.setJointMotorControl2(self.id, Agent.joint_indices[2], p.POSITION_CONTROL, targetPosition=rotation, force=force, maxVelocity=self.max_velocity_angular)
        while self.get_pose()[2] > math.degrees(rotation) + Agent.err_heading or self.get_pose()[2] < math.degrees(rotation) - Agent.err_heading:
            self.world_id.getCameraImage(320,200)
            self.world_id.stepSimulation()

        target_positions = [x_move, y_move, rotation]
        
        # set motor to move robot to next position (uses PD control)
        for i in range(2):
            self.world_id.setJointMotorControl2(self.id, Agent.joint_indices[i], p.POSITION_CONTROL,
                                    targetPosition=target_positions[i], force=force, maxVelocity=self.max_velocity,
                                    positionGain=0.05)
