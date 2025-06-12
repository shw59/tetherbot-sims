"""
unit_tests.py

This file defines a suite of unit tests in PyBullet for tetherbot simulation functionality.
"""

import pybullet as p
import pybullet_data
import math
import numpy as np

def make_tether(name, robot1_pos, robot2_pos, length_0, num_segments=10):
    """
    Create a tether and returns its corresponding id with length_0 meters and num_segments segments.
    """
    dy = length_0 / num_segments  # length of every segment along the tether between each pair of vertices
    dx = 0.01  # half the width of the tether
    lines = ["o tether"]
    # vertices
    for i in range(num_segments + 1):
        y = -dy * num_segments/2 + dy * i
        lines.append(f"v  { dx:.6f} {y:.6f} 0.000000")
        lines.append(f"v {-dx:.6f} {y:.6f} 0.000000")
    # faces
    for i in range(num_segments):
        a, b, c, d = 2*i+1, 2*i+3, 2*i+2, 2*i+4
        lines += [f"f {a} {b} {c}", f"f {c} {b} {d}"]

    tether_filename = f"{name}.obj"
    open(tether_filename, "w").write("\n".join(lines))

    # tether position should be midpoint of two robots
    tether_x = (robot1_pos[0] + robot2_pos[0])/2
    tether_y = (robot1_pos[1] + robot2_pos[1])/2

    tether_pos = [tether_x, tether_y, 0]

    first_x = robot1_pos[0]
    first_y = robot1_pos[1]
    second_x = robot2_pos[0]
    second_y = robot2_pos[1]
    diff_x = first_x - second_x
    diff_y = first_y - second_y

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

    orientation = p.getQuaternionFromEuler([0, 0, theta]) #[0, 0,-1*theta]

    id = p.loadSoftBody(tether_filename, 
                               basePosition = tether_pos, 
                               baseOrientation = orientation,
                               scale=1, 
                               mass=1., 
                               useNeoHookean=0, 
                               useBendingSprings=1,
                               useMassSpring=1, 
                               springElasticStiffness=30, 
                               springDampingStiffness=.9,
                               springDampingAllDirections=500, 
                               useSelfCollision=0, 
                               frictionCoeff=0, 
                               useFaceContact=1)
    
    # set tether color and appearance
    p.changeVisualShape(id, -1, rgbaColor=[1.0, 0.2, 0.58, 1.0], flags=p.VISUAL_SHAPE_DOUBLE_SIDED)

    return id

def anchor_tether(rope_id, first_id, second_id):
    """
    Anchors the ends of the given tether to which ever robot each end is closer to, 
    out of the two given robots
    """
    num_verts, *_ = p.getMeshData(rope_id, -1, flags=p.MESH_DATA_SIMULATION_MESH)

    p.createSoftBodyAnchor(rope_id, 0, first_id, 2)
    p.createSoftBodyAnchor(rope_id, 1, first_id, 2)
    p.createSoftBodyAnchor(rope_id, num_verts-2, second_id, 2)
    p.createSoftBodyAnchor(rope_id, num_verts-1, second_id, 2)
    
def make_robot(name, radius, position, heading=0, length=.01, mass=1.0, color=(0, 0.5, 1, 1)):
    """
    Creates a cylindrical robot object with specified radius and/or length, mass, and color and returns its corresponding id.
    """
    # inertia of a solid cylinder about its own center
    ixx = iyy = (1/12) * mass * (3 * radius**2 + length**2)
    izz = 0.5 * mass * radius**2

    rgba = " ".join(map(str, color))

    # set position of robot heading-indicator block
    heading_block_origin_x = radius / 2
    heading_block_origin_z = length / 2

    # set position of tether heading-indicator block
    tether_block_origin_z = length / 2

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
                <origin xyz="0 0 {length/2}" rpy="0 0 0"/>
                <geometry>
                    <cylinder length="{length}" radius="{radius}"/>
                </geometry>
                <material name="agent_color">
                    <color rgba="{rgba}"/>
                </material>
            </visual>

            <collision>
                <origin xyz="0 0 {length/2}" rpy="0 0 0"/>
                <geometry>
                    <cylinder length="{length}" radius="{radius}"/>
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
            <origin xyz="{heading_block_origin_x} 0 {heading_block_origin_z}" rpy="0 0 0"/>
        </joint>

        <link name="heading_block">
            <visual>
                <origin xyz="{heading_block_origin_x} 0 {heading_block_origin_z}" rpy="0 0 0"/>
                <geometry>
                    <box size="{radius} 0.01 0.01"/>
                </geometry>
                <material name="block_color"><color rgba="0 0 1 1"/></material>
            </visual>
        </link>
    </robot>
    """

    robot_blue_filename = f"{name}.urdf"
    open(robot_blue_filename, "w").write(urdf_text)

    id = p.loadURDF(robot_blue_filename, position)

    p.resetJointState(id, 2, math.radians(heading))

    return id

def set_straight_line(n, spacing):
    """
    Returns a list of positions that correspond to n robots seperated by "spacing" distance
    along the x-axis, centered about zero.
    """
    positions = []
    if (n%2 == 0):
        left = n/2
        right = n - left
        y = np.linspace(-left*spacing, right*spacing, n+1)
    else:
        left = round(n/2)
        right = n - left
        y = np.linspace(-left*spacing, right*spacing, n+1)
    for i in range(n):
        pos = [0, y[i], height]
        positions.append(pos)

    return positions

def get_tether_length(tether_id):
    """
    Return the length of the tether at any given time.
    The length is computed as the sum of the distances between pairs of vertices in the tether mesh.
    """
    n_verts, verts, *_ = p.getMeshData(tether_id, -1, flags=p.MESH_DATA_SIMULATION_MESH)

    length = 0.0
    for i in range(0, n_verts-3, 2):
        # 2-by-2 window: (i,i+1) || (i+2,i+3)
        p1 = [(verts[i][k]   + verts[i+1][k]) / 2.0 for k in range(3)]
        p2 = [(verts[i+2][k] + verts[i+3][k]) / 2.0 for k in range(3)]
        length += math.dist(p1, p2)
        
    return length

def get_robot_heading(robot_id):
    """
    Return heading vector [x, y] of a robot based on the position of its heading block.
    """
    robot_pos = p.getLinkState(robot_id, 2)[0]
    head_pos = p.getLinkState(robot_id, 3)[0]
    heading = [head_pos[i] - robot_pos[i] for i in range(3)]

    return heading[:2]

def get_tether_heading(robot_id, tether_id):
    """
    Return the heading vector [x, y] of the tether with respect to the given robot.
    """
    n_verts, verts, *_ = p.getMeshData(tether_id, -1, flags=p.MESH_DATA_SIMULATION_MESH)

    # get both end vertices of the tether
    p1 = [(verts[0][k] + verts[1][k]) / 2.0 for k in range(2)]
    p2 = [(verts[n_verts - 2][k] + verts[n_verts - 1][k]) / 2.0 for k in range(2)]

    # get the robot's position
    robot_pos = p.getLinkState(robot_id, 2)[0][:2]

    # check which vertex pair is closer and use the second to next pair to calculate heading
    dist1 = math.dist(robot_pos, p1)
    dist2 = math.dist(robot_pos, p2)
    if dist1 < dist2:
        p1_next = [(verts[1][k] + verts[2][k]) / 2.0 for k in range(2)] # second to end vertex
        heading = [p1_next[i] - robot_pos[i] for i in range(2)]
    else:
        p2_next = [(verts[n_verts - 3][k] + verts[n_verts - 2][k]) / 2.0 for k in range(2)]
        heading = [p2_next[i] - robot_pos[i] for i in range(2)]

    return heading

def get_tether_pos(tether_id):
    """
    Return the rough midpoint position of the tether object.
    """
    n_verts, verts, *_ = p.getMeshData(tether_id, -1, flags=p.MESH_DATA_SIMULATION_MESH)

    # find the index of the midpoint pair of vertices (upper if not exact middle)
    n_pairs = n_verts // 2
    mid_pair_idx = n_pairs // 2
    v1 = 2 * mid_pair_idx
    v2 = v1 + 1

    # return the midpoint of that pair (middle of the tether width)
    return [(verts[v1][k] + verts[v2][k]) / 2.0 for k in range(2)]

def get_theta(robot_id, tether_id):
    """
    Return the angle between the robot's heading and the tether's heading (in degrees).
    The angle is computed using the dot product.
    """
    hx, hy = get_robot_heading(robot_id)
    tx, ty = get_tether_heading(robot_id, tether_id)
    theta = math.atan2(hx*ty - hy*tx, hx*tx + hy*ty)

    return math.degrees(theta) % 360

def get_delta(robot_id, tether1_id, tether2_id=None):
    """
    Return the angle between the robot's heading and the tether's heading (in degrees).
    The angle is computed using the dot product.
    """
    hx, hy = get_robot_heading(robot_id)
    tx, ty = get_tether_heading(robot_id, tether1_id)
    theta1 = math.degrees(math.atan2(hx*ty - hy*tx, hx*tx + hy*ty))
    theta2 = 0
    if tether2_id is not None:
        tx2, ty2 = get_tether_heading(robot_id, tether2_id)
        theta2 = math.degrees(math.atan2(hx*ty2 - hy*tx2, hx*tx2 + hy*ty2))
    
    if theta1 < 0:
        theta1 = 360+theta1

    if theta2 < 0:
        theta2 = 360+theta2

    if round(theta2, 2) == 0:
        theta2 = 360
 
    delta = (theta1 - theta2)

    if delta < 0:
        delta = 360 + delta

    return delta

def smallest_signed_angle_diff(goal_angle, start_angle):
    """
    Computes the smallest rotation needed to go from one angle to another (+ indicates CCW, - indicates CW).
    """
    return (goal_angle - start_angle + math.pi) % (2 * math.pi) - math.pi

def normalize_vector(vec):
    norm = np.linalg.norm(vec)
    if norm == 0:
        return vec
    return vec / norm

def get_magnitude(vector):
    """
    Returns the magnitude a two-dimensional vector
    """
    return math.sqrt((vector[0]**2) + (vector[1]**2))

def move_robot(robot_id, target_pos, force=10):
    """
    Move the agent from its current position to a specified [x, y] target position in the world. The parameter
    target_pos should be a numpy array.
    """
    # amount to move (relative to base position)
    x_move, y_move = target_pos - np.array(p.getBasePositionAndOrientation(robot_id)[0][:2])

    # calculate rotation to face direction of movement
    base_heading = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(robot_id)[1])[2] # starting heading
    curr_heading = p.getJointState(robot_id, 2)[0]
    
    desired_h_x, desired_h_y = target_pos - np.array(p.getLinkState(robot_id, 2)[0][:2])
    desired_heading = math.atan2(desired_h_y, desired_h_x)

    # rotations must be fed into setJointMotorControl function with respect to the base heading, not current heading
    # use smallest signed angle difference and then add the current heading and base heading
    rotation = base_heading + curr_heading + (desired_heading - curr_heading + math.pi) % (2 * math.pi) - math.pi
    
    joint_indices = [1, 0, 2] # [x-direction, y-direction, rotation/heading]
    p.setJointMotorControlArray(robot_id, joint_indices, p.POSITION_CONTROL,
                                targetPositions=[x_move, y_move, rotation], forces=[force]*3)

def reached_target_position(robot_id, target_x, target_y):
    """
    Checks the robot's current position against a target position with given error tolerance. Returns true if robot has reached target.
    """
    return (p.getLinkState(robot_id, 2)[0][0] > target_x - err_pos and p.getLinkState(robot_id, 2)[0][0] < target_x + err_pos) and \
           (p.getLinkState(robot_id, 2)[0][1] > target_y - err_pos and p.getLinkState(robot_id, 2)[0][1] < target_y + err_pos)

def get_delta_vector(robot_id, tether1_id, tether2_id, goal_sig):
    """
    Calculates which direction the robot should move in so that it can
    attempt to reach its delta goal
    """
    delta = get_delta(robot_id, tether1_id, tether2_id)

    tether1_heading = get_tether_heading(robot_id, tether1_id)
    tether2_heading = get_tether_heading(robot_id, tether2_id)
    unit1 = normalize_vector(tether1_heading)
    unit2= normalize_vector(tether2_heading)
    summed_units = [unit1[0]+unit2[0], unit1[1]+unit2[1]]
    abs_value_of_summed = get_magnitude(summed_units)

    difference = goal_sig - delta

    abs_difference = abs(difference)

    vector = [0,0]

    if (abs_difference > err_delta):

        # as the unit vectors begin to cancel each other out (in 180 degree position), the angle vector will go to zero
        # not because the goal has been met, but because the unit vectors cancel. To avoid any stalling that the robot
        # may experience, I push the robot through 180 degree difference if that is not the goal degree difference of the
        # delta, which I determine from the first conditional in the if-statement below
        if (abs_difference > 10) and (-0.5 <= (unit1[0]+unit2[0]) <= 0.5) and (-0.5 <= (unit1[1]+unit2[1]) <= 0.5):

            hx, hy = get_robot_heading(robot_id)

            vector = [20*hx, 20*hy]
    
            # return vector
        else:
            if (delta < 180) and (delta > goal_sig):
                sign = -1
            elif (delta < 180) and (delta < goal_sig):
                sign = 1
            elif (delta > 180) and (delta < goal_sig):
                sign = -1
            else:
                sign = 1

            coefficient = sign*math.sqrt((abs_difference)/(10*math.pi))*(abs_value_of_summed)

            vector = [coefficient*summed_units[0], coefficient*summed_units[1]]

        return vector
    else:
        return vector

def get_strain_vector(robot_id, tether_id):
    """
    Calculates the vector direction that the robot should move to achieve the goal strain/tautness.
    """
    len = get_tether_length(tether_id)
    strain = (len - l_0) / l_0

    strain_diff = strain - goal_strain

    if strain_diff > 0:
        sign = 1
    elif strain_diff < 0:
        sign = -1
    else:
        sign = 0

    # The vector from the center of the robot to the point where the tether connects to the robot
    vector_norm = normalize_vector(np.array(get_tether_heading(robot_id, tether_id)))
    v_t = sign * (strain_diff ** 2) * vector_norm

    return v_t

def get_gradient_vector(robot_id):
    """
    Returns the vector pointing towards the target destination, where the vector's magnitude increases the farther from the
    destination the agent is.
    """
    curr_position = p.getLinkState(robot_id, 2)[0][:2]
    goal_position = goal_gradient
    distance = math.dist(curr_position, goal_position)
    if distance >= 10 * l_0:
        scale = 1
    elif distance >= 2 * l_0:
        scale = 0.5
    else:
        scale = 0.1
    
    v_g = scale * (1/distance) * (np.array(goal_position) - np.array(curr_position))

    return v_g

def get_repulsion_vector(sensor_info):
    """
    Returns the collision avoidance vector as a Gaussian curve given close-range sensor data.
    """
    amplitude = 3 * radius
    std_dev = radius
    v_r = np.array([0, 0])

    for i in range(len(sensor_info)):
        u_r, d, obj_type = sensor_info[i]
        v_r = v_r + amplitude * math.exp(-d**2 / (2 * std_dev**2)) * u_r
        # do something in the case that we can distinguish between robots, tethers, and obstacles

    return v_r

def get_closest_point_distance(robot_id, object_id, object_type):
    """
    Returns the closest point and distance from another object (tether, agent, or obstacle) to the agent as a tuple.
    """
    curr_pos = p.getLinkState(robot_id, 2)[0][:2]
    closest_points = []
    if object_type == "tether":
        _, verts, *_ = p.getMeshData(object_id, -1, flags=p.MESH_DATA_SIMULATION_MESH)
        closest_point = [float('inf'), float('inf')]
        nearest_dist = float('inf')
        for vert in verts:
            dist = math.dist(curr_pos, vert[:2])
            if dist < nearest_dist:
                nearest_dist = dist
                closest_point = vert[:2]
        return closest_point, nearest_dist
    elif object_type == "agent":
        closest_points = p.getClosestPoints(robot_id, object_id, float('inf'), linkIndexA=2, linkIndexB=2)
    elif object_type == "obstacle":
        closest_points = p.getClosestPoints(robot_id, object_id, float('inf'), linkIndexA=2, linkIndexB=-1)
    if closest_points:
        closest_point = closest_points[0][6][:2]
        nearest_dist = math.dist(curr_pos, closest_point)
        for point in closest_points:
            dist = math.dist(curr_pos, point[6][:2])
            if dist < nearest_dist:
                nearest_dist = dist
                closest_point = point[6][:2]
        return closest_point, nearest_dist
    return [float('inf'), float('inf')], float('inf')

def robot_sense(robot_id, objects, sensing_mode=0):
    """
    Three modes of sensing:
    0: Robot senses all obstacles, tethers, and robots indistinguishably. Returns list of tuples with unknown type classifications.
    1: Robot senses all obstacles, but can distinguish between them. Returns a list of tuples classified as "tether", "agent", or "obstacle".
    2: Robot can only sense robots and tethers, but not obstacles. 
    The objects list should be a list of tuples in the format (id, [x, y], "tether"/"agent"/"obstacle").
    Note: The heading vectors in the tuple return are numpy arrays.
    """
    curr_pos = np.array(p.getLinkState(robot_id, 2)[0][:2])
    sensor_data = []

    for i in range(len(objects)):
        obj_id, obj_pos, obj_type = objects[i]
        closest_point, dist = get_closest_point_distance(robot_id, obj_id, obj_type)
        if obj_id != robot_id and dist <= sensing_radius:
            if obj_type == "tether" and dist >= radius:
                heading_vec_norm = normalize_vector(curr_pos - np.array(closest_point))
            elif obj_type != "tether":
                heading_vec_norm = normalize_vector(curr_pos - np.array(obj_pos[:2]))
            else:
                continue

            if sensing_mode == 0:
                sensor_data.append((heading_vec_norm, dist, "unknown"))
            elif sensing_mode == 1:
                sensor_data.append((heading_vec_norm, dist, obj_type))
            elif sensing_mode == 2:
                if obj_type != "obstacle":
                    sensor_data.append((heading_vec_norm, dist, "unknown"))

    return sensor_data

def new_position_forward_with_strain(robot_id, tether_id):
    """
    Determines the new position the robot should move to to maintain its tether's goal strain based on 
    its heading and strain tether.
    """
    curr_pos = np.array(p.getLinkState(robot_id, 2)[0][:2])

    strain_vector = get_strain_vector(robot_id, tether_id)
    robot_heading = np.array(get_robot_heading(robot_id))

    resulting_vector = strain_weight * strain_vector + heading_weight * robot_heading
    
    new_position = curr_pos + resulting_vector
    
    return new_position

def new_position_gradient_with_strain(robot_id, tether_id):
    """
    Determines the new position the robot should move to to maintain its tether's goal strain and make progress
    towards its target position.
    """
    curr_position = np.array(p.getLinkState(robot_id, 2)[0][:2])

    strain_v = get_strain_vector(robot_id, tether_id)
    gradient_v = get_gradient_vector(robot_id)
    
    resulting_vector = strain_weight * strain_v + gradient_weight * gradient_v
    normalized_result = normalize_vector(resulting_vector)
    
    new_position = curr_position + 0.05 * normalized_result
    
    return new_position

def new_position_forward_with_repulsion(robot_id, sensor_data):
    """
    Determines the new position the robot should move to go forward but avoid obstacles in its sensing radius.
    """
    curr_pos = np.array(p.getLinkState(robot_id, 2)[0][:2])

    repulsion_vector = get_repulsion_vector(sensor_data)
    robot_heading = np.array(get_robot_heading(robot_id))

    resulting_vector = repulsion_weight * repulsion_vector + heading_weight * robot_heading
    
    new_position = curr_pos + resulting_vector
    
    return new_position

def new_position_forward_with_repulsion_strain(robot_id, tether_id, sensor_data):
    """
    Determines new position the robot should move to to maintain heading, strain, and also avoid obstacles if necessary.
    """
    curr_pos = np.array(p.getLinkState(robot_id, 2)[0][:2])

    strain_vector = get_strain_vector(robot_id, tether_id)
    repulsion_vector = get_repulsion_vector(sensor_data)
    robot_heading = np.array(get_robot_heading(robot_id))

    resulting_vector = strain_weight * strain_vector + heading_weight * robot_heading + repulsion_weight * repulsion_vector
    
    new_position = curr_pos + resulting_vector

    print(repulsion_vector, resulting_vector)
    
    return new_position

def new_position_angle_goal(robot_id, tether1_id, tether2_id, goal_delta):
    """
    Calculates where the given robot should move in order to get closer to
    attaining its delta goal.
    """
    direction_vector = get_delta_vector(robot_id, tether1_id, tether2_id, goal_delta)
    curr_x = p.getLinkState(robot_id, 2)[0][0]
    curr_y = p.getLinkState(robot_id, 2)[0][1]
    # normalized_result = normalize_vector(direction_vector)

    new_position = [curr_x+(0.01)*direction_vector[0], curr_y+(0.01)*direction_vector[1]]

    return new_position

def new_position_strain(robot_id, tether_id):
    """
    Determines the new position the robot should move to to maintain its tether's goal strain based on its tether strain.
    """
    curr_pos = np.array(p.getLinkState(robot_id, 2)[0][:2])

    strain_vector = get_strain_vector(robot_id, tether_id)

    resulting_vector = strain_weight * strain_vector
    
    new_position = curr_pos + resulting_vector
    
    return new_position

def new_position_gradient_with_angle_strain(robot_id, tether1_id, tether2_id, goal_delta):
    """
    This is to calculate the resulting vector of a robot that has two tethers, a goal angle, 
    and a global gradient.
    """
    curr_x = p.getLinkState(robot_id, 2)[0][0]
    curr_y = p.getLinkState(robot_id, 2)[0][1]
    strain_vector1 = get_strain_vector(robot_id, tether1_id)
    strain_vector2 = get_strain_vector(robot_id, tether2_id)
    tot_strain_vector = [strain_vector1[0]+strain_vector2[0], strain_vector1[1]+strain_vector2[1]]
    gradient = get_gradient_vector(robot_id)

    direction_vector = get_delta_vector(robot_id, tether1_id, tether2_id, goal_delta)

    resulting_vector = [strain_weight*tot_strain_vector[0]+gradient_weight*gradient[0]+angle_weight*direction_vector[0], 
                        strain_weight*tot_strain_vector[1]+gradient_weight*gradient[1]+angle_weight*direction_vector[1]]
    normalized_result = normalize_vector(resulting_vector)
    
    
    position = np.array([curr_x+(0.03)*normalized_result[0], curr_y+(0.03)*normalized_result[1]])

    return position

def new_position_2_tethered_robot(robot_id, tether1_id, tether2_id, sensor_data):
    """
    Determines new position the robot should move to to achieve goal angle, maintain gradient, strain, and also avoid obstacles if necessary.
    """
    curr_pos = np.array(p.getLinkState(robot_id, 2)[0][:2])

    angle_vector = np.array(get_delta_vector(robot_id, tether1_id, tether2_id, goal_delta))
    strain1_vector = get_strain_vector(robot_id, tether1_id)
    strain2_vector = get_strain_vector(robot_id, tether2_id)
    repulsion_vector = get_repulsion_vector(sensor_data)
    gradient_vector = get_gradient_vector(robot_id)

    resulting_vector = angle_weight * angle_vector + strain_weight * strain1_vector + strain_weight * strain2_vector \
                       + gradient_weight * gradient_vector + repulsion_weight * repulsion_vector
    
    new_position = curr_pos + 0.03 * normalize_vector(resulting_vector)
    
    return new_position

def new_position_1_tethered_robot(robot_id, tether_id, sensor_data):
    """
    Determines new position the robot should move to to maintain heading, strain, and also avoid obstacles if necessary.
    """
    curr_pos = np.array(p.getLinkState(robot_id, 2)[0][:2])

    strain_vector = get_strain_vector(robot_id, tether_id)
    repulsion_vector = get_repulsion_vector(sensor_data)
    gradient_vector = get_gradient_vector(robot_id)

    resulting_vector = strain_weight * strain_vector + gradient_weight * gradient_vector + repulsion_weight * repulsion_vector
    
    new_position = curr_pos + resulting_vector
    
    return new_position

"""UNIT TESTS"""

def waypoints_with_tether_test_ccw():
    # set initial object positions
    robot_blue_pos = [0, -1, 0.005]  # base position of the first robot
    robot_red_pos = [0, 0, 0.005]  # base position of the second robot

    # load objects
    robot_blue_id = make_robot("robot_blue", radius, robot_blue_pos)
    robot_red_id = make_robot("robot_red", radius, robot_red_pos, color=(1, 0, 0, 1))
    tether_id = make_tether("tether", robot_blue_pos, robot_red_pos, l_0, num_segments=20)
    cube_id = p.loadURDF("cube_small.urdf", [0, 0.5, .01])
    p.createConstraint(cube_id, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0.5, .01])

    # anchor the tether to the robots
    anchor_tether(tether_id, robot_blue_id, robot_red_id)

    # apply friction/damping between robots and the plane
    p.changeDynamics(robot_blue_id, -1, linearDamping=mu)
    p.changeDynamics(robot_red_id, -1, linearDamping=mu)

    waypoints = [(1, -1), (1, 1), (-1, 1), (-1, -1), (0, -1)]

    target_x, target_y, _ = p.getBasePositionAndOrientation(robot_blue_id)[0]
    idx = -1

    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)

        # calculate tether length and strain on every step
        l = get_tether_length(tether_id)
        strain = (l - l_0) / l_0

        # calculate tether angle relative to each robot's heading
        theta1 = get_theta(robot_blue_id, tether_id)
        theta2 = get_theta(robot_red_id, tether_id)

        # display results in the GUI
        p.addUserDebugText(f"tether length = {l:.2f} m\n tether strain = {strain:.2f}\n "
                            f"theta_blue = {theta1:.2f} deg\n theta_red = {theta2:.2f} deg",
                            [0, 0.5, 0.5], textColorRGB=[0, 0, 0], lifeTime=1)
    
        if reached_target_position(robot_blue_id, target_x, target_y) and (idx + 1) < len(waypoints):
            idx += 1
            target_x = waypoints[idx][0]
            target_y = waypoints[idx][1]
            move_robot(robot_blue_id, np.array([target_x, target_y]), force=25)

        p.stepSimulation()
        
def waypoints_with_tether_test_cw():
    # set initial object positions
    robot_blue_pos = [0, 1, 0.005]  # base position of the first robot
    robot_red_pos = [0, 0, 0.005]  # base position of the second robot

    # load objects
    robot_blue_id = make_robot("robot_blue", radius, robot_blue_pos)
    robot_red_id = make_robot("robot_red", radius, robot_red_pos, color=(1, 0, 0, 1))
    tether_id = make_tether("tether", robot_blue_pos, robot_red_pos, l_0, num_segments=20)
    cube_id = p.loadURDF("cube_small.urdf", [0, -0.5, .01])
    p.createConstraint(cube_id, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, -0.5, .01])

    # anchor the tether to the robots
    anchor_tether(tether_id, robot_blue_id, robot_red_id)

    # apply friction/damping between robots and the plane
    p.changeDynamics(robot_blue_id, -1, linearDamping=mu)
    p.changeDynamics(robot_red_id, -1, linearDamping=mu)

    waypoints = [(1, 1), (1, -1), (-1, -1), (-1, 1), (0, 1)]
    target_x, target_y, _ = p.getBasePositionAndOrientation(robot_blue_id)[0]
    idx = -1

    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)

        # calculate tether length and strain on every step
        l = get_tether_length(tether_id)
        strain = (l - l_0) / l_0

        # calculate tether angle relative to each robot's heading
        theta1 = get_theta(robot_blue_id, tether_id)
        theta2 = get_theta(robot_red_id, tether_id)

        # display results in the GUI
        p.addUserDebugText(f"tether length = {l:.2f} m\n tether strain = {strain:.2f}\n "
                            f"theta_blue = {theta1:.2f} deg\n theta_red = {theta2:.2f} deg",
                            [0, 0.5, 0.5], textColorRGB=[0, 0, 0], lifeTime=1)
    
        if reached_target_position(robot_blue_id, target_x, target_y) and (idx + 1) < len(waypoints):
            idx += 1
            target_x = waypoints[idx][0]
            target_y = waypoints[idx][1]
            move_robot(robot_blue_id, np.array([target_x, target_y]), force=25)

        p.stepSimulation()

def maintain_strain_heading_test():
    # set initial object positions
    robot_blue_pos = [0, -1, 0.005]  # base position of the first robot
    robot_red_pos = [0, 0, 0.005]  # base position of the second robot

    # load objects
    robot_blue_id = make_robot("robot_blue", radius, robot_blue_pos)
    robot_red_id = make_robot("robot_red", radius, robot_red_pos, color=(1, 0, 0, 1))
    tether_id = make_tether("tether", robot_blue_pos, robot_red_pos, l_0, num_segments=20)

    # anchor the tether to the robots
    anchor_tether(tether_id, robot_blue_id, robot_red_id)

    # apply friction/damping between robots and the plane
    p.changeDynamics(robot_blue_id, -1, linearDamping=mu)
    p.changeDynamics(robot_red_id, -1, linearDamping=mu)

    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)

        # calculate tether length and strain on every step
        l = get_tether_length(tether_id)
        strain = (l - l_0) / l_0

        # calculate tether angle relative to each robot's heading
        theta1 = get_theta(robot_blue_id, tether_id)
        theta2 = get_theta(robot_red_id, tether_id)

        # display results in the GUI
        p.addUserDebugText(f"tether length = {l:.2f} m\n tether strain = {strain:.2f}\n "
                            f"theta_blue = {theta1:.2f} deg\n theta_red = {theta2:.2f} deg",
                            [0, 0.5, 0.5], textColorRGB=[0, 0, 0], lifeTime=1)
    
        if reached_target_position(robot_blue_id, robot_blue_pos[0], robot_blue_pos[1]):
            robot_blue_pos = new_position_forward_with_strain(robot_blue_id, tether_id)
            move_robot(robot_blue_id, robot_blue_pos, force=60)

        p.stepSimulation() 

def maintain_strain_heading_test_2():
    # set initial object positions
    robot_blue_pos = [0, -1, 0.005]  # base position of the first robot
    robot_red_pos = [0, 0, 0.005]  # base position of the second robot

    # load objects
    robot_blue_id = make_robot("robot_blue", radius, robot_blue_pos)
    robot_red_id = make_robot("robot_red", radius, robot_red_pos, color=(1, 0, 0, 1))
    tether_id = make_tether("tether", robot_blue_pos, robot_red_pos, l_0, num_segments=20)
    cube_id = p.loadURDF("cube_small.urdf", [1, -0.5, .01])
    p.createConstraint(cube_id, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [1, -0.5, .01])

    # anchor the tether to the robots
    anchor_tether(tether_id, robot_blue_id, robot_red_id)

    # apply friction/damping between robots and the plane
    p.changeDynamics(robot_blue_id, -1, linearDamping=mu)
    p.changeDynamics(robot_red_id, -1, linearDamping=mu)

    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)

        # calculate tether length and strain on every step
        l = get_tether_length(tether_id)
        strain = (l - l_0) / l_0

        # calculate tether angle relative to each robot's heading
        theta1 = get_theta(robot_blue_id, tether_id)
        theta2 = get_theta(robot_red_id, tether_id)

        # display results in the GUI
        p.addUserDebugText(f"tether length = {l:.2f} m\n tether strain = {strain:.2f}\n "
                            f"theta_blue = {theta1:.2f} deg\n theta_red = {theta2:.2f} deg",
                            [0, 0.5, 0.5], textColorRGB=[0, 0, 0], lifeTime=1)
    
        if reached_target_position(robot_blue_id, robot_blue_pos[0], robot_blue_pos[1]):
            robot_blue_pos = new_position_forward_with_strain(robot_blue_id, tether_id)
            move_robot(robot_blue_id, robot_blue_pos, force=60)

        if reached_target_position(robot_red_id, robot_red_pos[0], robot_red_pos[1]):
            robot_red_pos = new_position_forward_with_strain(robot_red_id, tether_id)
            move_robot(robot_red_id, robot_red_pos, force=60)

        p.stepSimulation()

def maintain_strain_gradient_test():
    # set initial object positions
    robot_blue_pos = [0, -1, 0.005]  # base position of the first robot
    robot_red_pos = [0, 0, 0.005]  # base position of the second robot

    # load objects
    robot_blue_id = make_robot("robot_blue", radius, robot_blue_pos)
    robot_red_id = make_robot("robot_red", radius, robot_red_pos, color=(1, 0, 0, 1))
    tether_id = make_tether("tether", robot_blue_pos, robot_red_pos, l_0, num_segments=20)

    # anchor the tether to the robots
    anchor_tether(tether_id, robot_blue_id, robot_red_id)

    # apply friction/damping between robots and the plane
    p.changeDynamics(robot_blue_id, -1, linearDamping=mu)
    p.changeDynamics(robot_red_id, -1, linearDamping=mu)

    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)

        # calculate tether length and strain on every step
        l = get_tether_length(tether_id)
        strain = (l - l_0) / l_0

        # calculate tether angle relative to each robot's heading
        theta1 = get_theta(robot_blue_id, tether_id)
        theta2 = get_theta(robot_red_id, tether_id)

        # display results in the GUI
        p.addUserDebugText(f"tether length = {l:.2f} m\n tether strain = {strain:.2f}\n "
                            f"theta_blue = {theta1:.2f} deg\n theta_red = {theta2:.2f} deg",
                            [0, 0.5, 0.5], textColorRGB=[0, 0, 0], lifeTime=1)
    
        if reached_target_position(robot_blue_id, robot_blue_pos[0], robot_blue_pos[1]):
            robot_blue_pos = new_position_gradient_with_strain(robot_blue_id, tether_id)
            move_robot(robot_blue_id, robot_blue_pos, force=60)

        if reached_target_position(robot_red_id, robot_red_pos[0], robot_red_pos[1]): 
            robot_red_pos = new_position_gradient_with_strain(robot_red_id, tether_id)
            move_robot(robot_red_id, robot_red_pos, force=60)

        p.stepSimulation()

def maintain_forward_avoid_collision_test():
    # set initial object positions
    robot_blue_pos = [0, 0, 0.005]  # base position of the first robot
    robot_red_pos = [0, 2, 0.005]  # base position of the second robot
    cube_pos = [1.5, 1, 0.5]

    # load objects
    robot_blue_id = make_robot("robot_blue", radius, robot_blue_pos, 90)
    robot_red_id = make_robot("robot_red", radius, robot_red_pos, color=(1, 0, 0, 1))
    tether_id = make_tether("tether", [-0.5, -2, 0.005], [0.5, -2, 0.005], 1)
    cube_id = p.loadURDF("cube.urdf", cube_pos)
    p.createConstraint(cube_id, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], cube_pos)

    # query list of object ids, positions, and type
    obj_list = [(robot_blue_id, robot_blue_pos[:2], "agent"), (robot_red_id, robot_red_pos[:2], "agent"),
                (cube_id, cube_pos[:2], "obstacle"), (tether_id, [0, -2], "tether")]

    # apply friction/damping between robots and the plane
    p.changeDynamics(robot_blue_id, -1, linearDamping=mu)
    p.changeDynamics(robot_red_id, -1, linearDamping=mu)

    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)

        sensor_data = robot_sense(robot_blue_id, obj_list, 2)
    
        if reached_target_position(robot_blue_id, robot_blue_pos[0], robot_blue_pos[1]):
            robot_blue_pos = new_position_forward_with_repulsion(robot_blue_id, sensor_data)
            move_robot(robot_blue_id, robot_blue_pos, force=60)

        p.stepSimulation()

def maintain_strain_heading_repulsion_test():
    # set initial object positions
    robot_blue_pos = [-3, -1, 0.005]  # base position of the first robot
    robot_red_pos = [-3, 0, 0.005]  # base position of the second robot
    cube_pos = [0, -0.5, 0.5]

    # load objects
    robot_blue_id = make_robot("robot_blue", radius, robot_blue_pos)
    robot_red_id = make_robot("robot_red", radius, robot_red_pos, color=(1, 0, 0, 1))
    tether_id = make_tether("tether", robot_blue_pos, robot_red_pos, l_0, num_segments=20)
    cube_id = p.loadURDF("cube.urdf", cube_pos)
    p.createConstraint(cube_id, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], cube_pos)

    # anchor the tether to the robots
    anchor_tether(tether_id, robot_blue_id, robot_red_id)

    # apply friction/damping between robots and the plane
    p.changeDynamics(robot_blue_id, -1, linearDamping=mu)
    p.changeDynamics(robot_red_id, -1, linearDamping=mu)

    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)

        # calculate tether length and strain on every step
        l = get_tether_length(tether_id)
        strain = (l - l_0) / l_0

        # calculate tether angle relative to each robot's heading
        theta1 = get_theta(robot_blue_id, tether_id)
        theta2 = get_theta(robot_red_id, tether_id)

        # query list of object ids, positions, and type
        obj_list = [(robot_blue_id, p.getLinkState(robot_blue_id, 2)[0][:2], "agent"), 
                    (robot_red_id, p.getLinkState(robot_red_id, 2)[0][:2], "agent"),
                    (cube_id, p.getBasePositionAndOrientation(cube_id)[0][:2], "obstacle"), 
                    (tether_id, [0, 0], "tether")]
        
        sensor_data = robot_sense(robot_blue_id, obj_list, 1)

        print(sensor_data)

        # display results in the GUI
        p.addUserDebugText(f"tether length = {l:.2f} m\n tether strain = {strain:.2f}\n "
                            f"theta_blue = {theta1:.2f} deg\n theta_red = {theta2:.2f} deg",
                            [0, 0.5, 0.5], textColorRGB=[0, 0, 0], lifeTime=1)
            
        if reached_target_position(robot_blue_id, robot_blue_pos[0], robot_blue_pos[1]):
            robot_blue_pos = new_position_forward_with_repulsion_strain(robot_blue_id, tether_id, sensor_data)
            move_robot(robot_blue_id, robot_blue_pos, force=60)

        if reached_target_position(robot_red_id, robot_red_pos[0], robot_red_pos[1]):
            robot_red_pos = new_position_forward_with_repulsion_strain(robot_red_id, tether_id, sensor_data)
            move_robot(robot_red_id, robot_red_pos, force=60)
        p.stepSimulation()

def achieve_goal_angle_test():
    N = 3

    # set initial object positions
    # initial_robot_positions = set_straight_line(N, l_0)
    
    initial_robot_positions = [[0, 0, height],
                               [0, 1, height],
                               [1, 1, height]]
    # initial_robot_positions = [[1,1,height]]
    # initial_robot_positions = set_straight_line(N, l_0)
    
    # list of x-y current target positions for each agent (starts at their initial positions)
    target_pos = [initial_robot_positions[i][:2] for i in range(len(initial_robot_positions))]

    # a list of all of the agent objects created
    robot_ids = []

    # a list of tether objects
    tether_ids = []

    # populates the list of robot objects with robot objects
    for i in range(N):
        robot_ids.append(make_robot(f"robot{i}", radius, initial_robot_positions[i]))

    # applies friction/damping between robots and the plane
    for i in range(N):
        p.changeDynamics(robot_ids[i], -1, linearDamping=mu)

    # populates the list of tether objects with tether objects
    for i in range(N-1):
        tether_ids.append(make_tether(f"tether{i}", initial_robot_positions[i], initial_robot_positions[i+1], l_0, num_segments=1))

    # anchors all of the tethers to their respective robots
    for i in range(N-1):
        anchor_tether(tether_ids[i], robot_ids[i], robot_ids[i+1])
        
    runs = 0

    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)

        if runs%100 == 0:
            # calculate tether angle relative to each robot's heading
            delta1 = get_delta(robot_ids[1], tether_ids[0], tether_ids[1])
            # theta2 = get_theta(robot_ids[1], tether_ids[0])

            # calculate tether length and strain on every step
            l1 = get_tether_length(tether_ids[0])
            strain1 = (l1 - l_0) / l_0
            
            l2 = get_tether_length(tether_ids[1])
            strain2 = (l2 - l_0) / l_0

            # display results in the GUI
            p.addUserDebugText(f"delta = {delta1:.2f} deg \n strain1 = {strain1:.2f} \n strain2 = {strain2:.2f}",
                                [0, 0.5, 0.5], textColorRGB=[0, 0, 0], lifeTime=1)

        if reached_target_position(robot_ids[1], target_pos[1][0], target_pos[1][1]):
            new_pos = new_position_angle_goal(robot_ids[1], tether_ids[0], tether_ids[1], goal_delta)
            target_pos[1] = (new_pos[0], new_pos[1])
            move_robot(robot_ids[1], np.array(target_pos[1]), force=60)

        if reached_target_position(robot_ids[0], target_pos[0][0], target_pos[0][1]):
            new_pos = new_position_strain(robot_ids[0], tether_ids[0])
            target_pos[0] = (new_pos[0], new_pos[1])
            move_robot(robot_ids[0], np.array(target_pos[0]), force=60)

        if reached_target_position(robot_ids[2], target_pos[2][0], target_pos[2][1]):
            new_pos = new_position_strain(robot_ids[2], tether_ids[1])
            target_pos[2] = (new_pos[0], new_pos[1])
            move_robot(robot_ids[2], np.array(target_pos[2]), force=60)

        runs = runs + 1

        p.stepSimulation()

def achieve_goal_angle_with_gradient_strain_test():
    N = 3

    # set initial object positions
    # initial_robot_positions = set_straight_line(N, l_0)
    
    initial_robot_positions = [[0, 0,height],
                               [0, 1, height],
                               [1, 1, height]]
    # initial_robot_positions = [[1,1,height]]
    # initial_robot_positions = set_straight_line(N, l_0)
    
    # list of x-y current target positions for each agent (starts at their initial positions)
    target_pos = [initial_robot_positions[i][:2] for i in range(len(initial_robot_positions))]

    # a list of all of the agent objects created
    robot_ids = []

    # a list of tether objects
    tether_ids = []

    # populates the list of robot objects with robot objects
    for i in range(N):
        robot_ids.append(make_robot(f"robot{i}", radius, initial_robot_positions[i]))

    # applies friction/damping between robots and the plane
    for i in range(N):
        p.changeDynamics(robot_ids[i], -1, linearDamping=mu)

    # populates the list of tether objects with tether objects
    for i in range(N-1):
        tether_ids.append(make_tether(f"tether{i}", initial_robot_positions[i], initial_robot_positions[i+1], l_0, num_segments=1))

    # anchors all of the tethers to their respective robots
    for i in range(N-1):
        anchor_tether(tether_ids[i], robot_ids[i], robot_ids[i+1])
        
    runs = 0

    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)

        if runs%100 == 0:
            # calculate tether angle relative to each robot's heading
            delta1 = get_delta(robot_ids[1], tether_ids[0], tether_ids[1])
            # theta2 = get_theta(robot_ids[1], tether_ids[0])
            
            # calculate tether length and strain on every step
            l1 = get_tether_length(tether_ids[0])
            strain1 = (l1 - l_0) / l_0
            
            l2 = get_tether_length(tether_ids[1])
            strain2 = (l2 - l_0) / l_0

            # display results in the GUI
            p.addUserDebugText(f"delta = {delta1:.2f} deg \n strain1 = {strain1:.2f} \n strain2 = {strain2:.2f}",
                                [0, 0.5, 0.5], textColorRGB=[0, 0, 0], lifeTime=1)

        if reached_target_position(robot_ids[1], target_pos[1][0], target_pos[1][1]):
            new_pos = new_position_gradient_with_angle_strain(robot_ids[1], tether_ids[0], tether_ids[1], goal_delta)
            target_pos[1] = (new_pos[0], new_pos[1])
            move_robot(robot_ids[1], np.array(target_pos[1]), force=60)

        if reached_target_position(robot_ids[0], target_pos[0][0], target_pos[0][1]):
            new_pos = new_position_gradient_with_strain(robot_ids[0], tether_ids[0])
            target_pos[0] = (new_pos[0], new_pos[1])
            move_robot(robot_ids[0], np.array(target_pos[0]), force=60)

        if reached_target_position(robot_ids[2], target_pos[2][0], target_pos[2][1]):
            new_pos = new_position_gradient_with_strain(robot_ids[2], tether_ids[1])
            target_pos[2] = (new_pos[0], new_pos[1])
            move_robot(robot_ids[2], np.array(target_pos[2]), force=60)

        runs = runs + 1

        p.stepSimulation()

def all_vectors_test():
    N = 3

    # set initial object positions
    # initial_robot_positions = set_straight_line(N, l_0)
    
    initial_robot_positions = [[0, 0,height],
                               [0, 1, height],
                               [1, 1, height]]
    # initial_robot_positions = [[1,1,height]]
    # initial_robot_positions = set_straight_line(N, l_0)

    cube_pos = [1, 0, 0.5]
    
    # list of x-y current target positions for each agent (starts at their initial positions)
    target_pos = [initial_robot_positions[i][:2] for i in range(len(initial_robot_positions))]

    # a list of all of the agent objects created
    robot_ids = []

    # a list of tether objects
    tether_ids = []

    # populates the list of robot objects with robot objects
    for i in range(N):
        robot_ids.append(make_robot(f"robot{i}", radius, initial_robot_positions[i]))

    # applies friction/damping between robots and the plane
    for i in range(N):
        p.changeDynamics(robot_ids[i], -1, linearDamping=mu)

    # populates the list of tether objects with tether objects
    for i in range(N-1):
        tether_ids.append(make_tether(f"tether{i}", initial_robot_positions[i], initial_robot_positions[i+1], l_0, num_segments=1))

    # anchors all of the tethers to their respective robots
    for i in range(N-1):
        anchor_tether(tether_ids[i], robot_ids[i], robot_ids[i+1])

    cube_id = p.loadURDF("cube.urdf", cube_pos)
    p.createConstraint(cube_id, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], cube_pos)

    # query list of object ids, positions, and type
    obj_list = [(robot_ids[i], p.getLinkState(robot_ids[i], 2)[0][:2], "agent") for i in range(3)]
    obj_list.append((cube_id, cube_pos, "obstacle"))
    for i in range(2):
        obj_list.append((tether_ids[i], [0, 0], "tether"))
        
    runs = 0

    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)

        if runs%100 == 0:
            # calculate tether angle relative to each robot's heading
            delta1 = get_delta(robot_ids[1], tether_ids[0], tether_ids[1])
            # theta2 = get_theta(robot_ids[1], tether_ids[0])
            
            # calculate tether length and strain on every step
            l1 = get_tether_length(tether_ids[0])
            strain1 = (l1 - l_0) / l_0
            
            l2 = get_tether_length(tether_ids[1])
            strain2 = (l2 - l_0) / l_0

            # display results in the GUI
            p.addUserDebugText(f"delta = {delta1:.2f} deg \n strain1 = {strain1:.2f} \n strain2 = {strain2:.2f}",
                                [0, 0.5, 0.5], textColorRGB=[0, 0, 0], lifeTime=1)
            
        sensor_data = []
        for i in range(3):
            sensor_data.append(robot_sense(robot_ids[i], obj_list, 1))

        if reached_target_position(robot_ids[1], target_pos[1][0], target_pos[1][1]):
            new_pos = new_position_2_tethered_robot(robot_ids[1], tether_ids[0], tether_ids[1], sensor_data[1])
            target_pos[1] = (new_pos[0], new_pos[1])
            move_robot(robot_ids[1], np.array(target_pos[1]), force=60)

        if reached_target_position(robot_ids[0], target_pos[0][0], target_pos[0][1]):
            new_pos = new_position_1_tethered_robot(robot_ids[0], tether_ids[0], sensor_data[0])
            target_pos[0] = (new_pos[0], new_pos[1])
            move_robot(robot_ids[0], np.array(target_pos[0]), force=60)

        if reached_target_position(robot_ids[2], target_pos[2][0], target_pos[2][1]):
            new_pos = new_position_1_tethered_robot(robot_ids[2], tether_ids[1], sensor_data[2])
            target_pos[2] = (new_pos[0], new_pos[1])
            move_robot(robot_ids[2], np.array(target_pos[2]), force=60)

        runs = runs + 1

        p.stepSimulation()

    
GRAVITYZ = -9.81  # m/s^2

radius = 0.1  # diameter of each robot in meters
mass = 1.0 # mass of each robot in kg
height = 0.005 # height of each robot
mu = 2.5  # friction coefficient between robots and plane
sensing_radius = radius * 4

# tether properties
l_0 = 1   # unstretched/taut length of tether in meters
goal_strain = 0.1
goal_gradient = [2, 2]
goal_delta = 90

# vector weights
angle_weight = 5
strain_weight = 6
heading_weight = 2
gradient_weight = 4
repulsion_weight = 10

gradient_target = [2, 2]
err_pos = 0.01 # positional error tolerance
err_delta = 5 # delta error tolerance

debugging = False  # set to True to print vertex positions of tether

def main():
    p.connect(p.GUI) # connect to PyBullet GUI
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) # add pybullet_data to search path

    # set parameters
    p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI,0) # disable side bar windows in the GUI
    p.setGravity(0, 0, GRAVITYZ)
    p.setTimeStep(1./240.)

    # load plane
    p.loadURDF("plane.urdf")

    # wall dimensions
    wall_height = 0.2
    thickness = 0.01
    half_length = 5
    half_width = 7

    # create boundary
    def create_wall(pos, half_extents):
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

    # +X wall
    create_wall([half_length, 0, wall_height / 2], [thickness, half_width, wall_height / 2])
    # -X wall
    create_wall([-half_length, 0, wall_height / 2], [thickness, half_width, wall_height / 2])
    # +Y wall
    create_wall([0, half_width, wall_height / 2], [half_length, thickness, wall_height / 2])
    # -Y wall
    create_wall([0, -half_width, wall_height / 2], [half_length, thickness, wall_height / 2])

    """RUN UNIT TESTS (uncomment the one you want to run)"""
    # waypoints_with_tether_test_ccw()
    # waypoints_with_tether_test_cw()
    # maintain_strain_heading_test()
    # maintain_strain_heading_test_2()
    # maintain_strain_gradient_test()
    # maintain_forward_avoid_collision_test()
    # maintain_strain_heading_repulsion_test()
    # achieve_goal_angle_test()
    # achieve_goal_angle_with_gradient_strain_test()
    all_vectors_test()

if __name__ == "__main__":
  main()