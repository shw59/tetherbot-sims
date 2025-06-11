"""
tetherbots.py

This file defines a PyBullet simulation of two disk-shaped controllable robots connected by a flexible tether.
"""

import pybullet as p
import pybullet_data
import math
import numpy as np

def make_tether(robot1_pos, robot2_pos, length_0, num_segments=10):
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

    tether_filename = f"tether.obj"
    open(tether_filename, "w").write("\n".join(lines))

    # tether position should be midpoint of the two robots
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
                               springElasticStiffness=.1, 
                               springDampingStiffness=.1,
                               springDampingAllDirections=1, 
                               useSelfCollision=0, 
                               frictionCoeff=0, 
                               useFaceContact=1)
    
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
    
def make_robot(radius, position, heading=0, length=.01, mass=1.0, color=(0, 0.5, 1, 1)):
    """
    Returns the id of a cylindrical robot object with specified position, heading (degrees), radius and/or length, mass, and color.
    """
    # inertia of a solid cylinder about its own center
    ixx = iyy = (1/12) * mass * (3 * radius**2 + length**2)
    izz = 0.5 * mass * radius**2

    rgba = " ".join(map(str, color))

    # set position of heading-indicator block
    block_origin_x = radius / 2
    block_origin_z = length / 2

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
    
    robot_blue_filename = f"robot.urdf"
    open(robot_blue_filename, "w").write(urdf_text)

    id = p.loadURDF(robot_blue_filename, position)

    p.resetJointState(id, 2, math.radians(heading)) # set robot heading

    return id

def get_tether_length(tether_id):
    """
    Return the length of the tether at any given time.
    The length is computed as the sum of the distances between pairs of vertices in the tether mesh.
    """
    n_verts, verts, *_ = p.getMeshData(tether_id, -1, flags=p.MESH_DATA_SIMULATION_MESH)

    length = 0.0
    for i in range(0, n_verts-3, 2):
        p1 = [(verts[i][k]   + verts[i+1][k]) / 2.0 for k in range(3)] # midpoints between the vertices
        p2 = [(verts[i+2][k] + verts[i+3][k]) / 2.0 for k in range(3)]
        length += math.dist(p1, p2)
        
    return length

def get_robot_heading(robot_id):
    """
    Return heading vector [x, y] of a robot based on the position of its heading block.
    """
    robot_pos = p.getLinkState(robot_id, 2)[0]
    head_pos = p.getLinkState(robot_id, 3)[0]
    heading = [head_pos[i] - robot_pos[i] for i in range(2)]

    return heading

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

def get_theta(robot_id, tether_id):
    """
    Return the angle between the robot's heading and the tether's heading (in degrees).
    The angle is computed using the dot product.
    """
    hx, hy = get_robot_heading(robot_id)
    tx, ty = get_tether_heading(robot_id, tether_id)
    theta = math.atan2(hx*ty - hy*tx, hx*tx + hy*ty)

    return math.degrees(theta) % 360
  
def smallest_signed_angle_diff(goal_angle, start_angle):
    """
    Computes the smallest rotation needed to go from one angle to another (+ indicates CCW, - indicates CW).
    """
    return (goal_angle - start_angle + math.pi) % (2 * math.pi) - math.pi

def move_robot(robot_id, x, y, force=10):
    """
    Move the robot to a specified position (x, y) with an optionally specified force and positional error tolerance.
    """
    # amount to move (relative to base position)
    x_move = x - p.getBasePositionAndOrientation(robot_id)[0][0]
    y_move = y - p.getBasePositionAndOrientation(robot_id)[0][1]

    # calculate rotation to face direction of movement
    base_heading = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(robot_id)[1])[2] # starting heading
    
    curr_heading = p.getJointState(robot_id, 2)[0]

    x_curr_pos, y_curr_pos = p.getLinkState(robot_id, 2)[0][:2]
    desired_heading = math.atan2(y - y_curr_pos, x - x_curr_pos)

    # rotations must be fed into setJointMotorControl function with respect to the base heading, not current heading
    rotation = base_heading + curr_heading + smallest_signed_angle_diff(desired_heading, curr_heading)
    
    joint_indices = [1, 0, 2] # [x-direction, y-direction, rotation/heading]
    p.setJointMotorControlArray(robot_id, joint_indices, p.POSITION_CONTROL,
                                targetPositions=[x_move, y_move, rotation], forces=[force]*3)
    
def reached_target_position(robot_id, target_x, target_y, err):
    """
    Checks the robot's current position against a target position with given error tolerance. Returns true if robot has reached target.
    """
    return (p.getLinkState(robot_id, 2)[0][0] > target_x - err and p.getLinkState(robot_id, 2)[0][0] < target_x + err) and \
           (p.getLinkState(robot_id, 2)[0][1] > target_y - err and p.getLinkState(robot_id, 2)[0][1] < target_y + err)

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

def get_strain_vector(robot_id, tether_id):
    """
    Calculates the vector direction that the robot should move to achieve the goal strain.
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
    vector = get_tether_heading(robot_id, tether_id)
    length_of_vector = math.sqrt((vector[0]**2) + (vector[1]**2))
    unit_vector = [(1/length_of_vector)*vector[0], (1/length_of_vector)*vector[1]]

    V_t = [sign*(strain_diff**2)*unit_vector[0], sign*(strain_diff**2)*unit_vector[1]]

    return V_t

def get_sigma(robot_id, tether1_id, tether2_id=None):
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
 
    sigma = (theta1 - theta2)

    if sigma < 0:
        sigma = 360 + sigma

    return sigma

def get_sigma_vector(robot_id, tether1_id, tether2_id, goal_sig):
    """
    Calculates which direction the robot should move in so that it can
    attempt to reach its sigma goal
    """
    sigma = get_sigma(robot_id, tether1_id, tether2_id)

    tether1_heading = get_tether_heading(robot_id, tether1_id)
    tether2_heading = get_tether_heading(robot_id, tether2_id)
    unit1 = normalize_vector(tether1_heading)
    unit2= normalize_vector(tether2_heading)
    summed_units = [unit1[0]+unit2[0], unit1[1]+unit2[1]]
    abs_value_of_summed = get_magnitude(summed_units)

    difference = goal_sig - sigma

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
            if (sigma < 180) and (sigma > goal_sig):
                sign = -1
            elif (sigma < 180) and (sigma < goal_sig):
                sign = 1
            elif (sigma > 180) and (sigma < goal_sig):
                sign = -1
            else:
                sign = 1

            coefficient = sign*math.sqrt((abs_difference)/(10*math.pi))*(abs_value_of_summed)

            vector = [coefficient*summed_units[0], coefficient*summed_units[1]]

        return vector
    else:
        return vector

    # if abs_difference > err_delta:
    #     if ((difference <= -10) or (difference >= 10)) and (-0.5 <= (unit1[0]+unit2[0]) <= 0.5) and (-0.5 <= (unit1[1]+unit2[1]) <= 0.5):
            
    #         hx, hy = get_robot_heading(robot_id)
    #         vector = [20*hx, 20*hy]
    
    #         return vector
    #     elif (sigma < 0) and (goal_sig < 0) and (sigma < goal_sig):
    #         sign = -1

    #         coefficient = sign*math.sqrt((abs_difference)/(10*math.pi))*(abs_value_of_summed)

    #         vector = [coefficient*summed_units[0], coefficient*summed_units[1]]
            
    #         return vector
    #     else:
    #         if difference < 0:
    #             sign = -1
    #         else:
    #             sign = 1
            
    #         # if (difference >= 0 ) and (difference < 90):
    #         #     sign = -1*sign

    #         # if (difference <= 0) and (difference > -90):
    #         #     sign = -1*sign

    #         # if sigma > 180:
    #         #     sign = -1*sign

    #         # if (90 < sigma < 270):
    #         #     sign = -1*sign
    #         # if sigma < difference:
    #         #     sign = 1
    #         # else:
    #         #     sign = 1
            
    #         # print(sigma)
    #         # print(sign)

            
            
    #         coefficient = sign*math.sqrt((abs_difference)/(10*math.pi))*(abs_value_of_summed)

    #         vector = [coefficient*summed_units[0], coefficient*summed_units[1]]

    #         # print("sigma: " + str(sigma))
    #         # print("difference: " + str(difference))
    #         # print("vector: " + str(vector))
    #         # print("\n")
            
    #         return vector
    # else:
    #     # print("sigma: " + str(sigma))
    #     # print("difference: " + str(difference))
    #     # print("vector: " + str(vector))
    #     # print("\n")

    #     return vector

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
    match object_type:
        case "tether":
            _, verts, *_ = p.getMeshData(object_id, -1, flags=p.MESH_DATA_SIMULATION_MESH)
            closest_point = [float('inf'), float('inf')]
            nearest_dist = float('inf')
            for vert in verts:
                dist = math.dist(curr_pos, vert[:2])
                if dist < nearest_dist:
                    nearest_dist = dist
                    closest_point = vert[:2]
            return closest_point, nearest_dist
        case "agent":
            closest_points = p.getClosestPoints(robot_id, object_id, float('inf'), linkIndexA=2, linkIndexB=2)
        case "obstacle":
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
                heading_vec_norm = normalize_vector(curr_pos - np.array(obj_pos))
            else:
                continue

            match sensing_mode:
                case 0:
                    sensor_data.append((heading_vec_norm, dist, "unknown"))
                case 1:
                    sensor_data.append((heading_vec_norm, dist, obj_type))
                case 2:
                    if obj_type != "obstacle":
                        sensor_data.append((heading_vec_norm, dist, "unknown"))

    return sensor_data
    
def new_position_for_sigma_goal(robot_id, tether1_id, tether2_id, goal_sigma):
    """
    Calculates where the given robot should move in order to get closer to
    attaining its sigma goal.
    """
    direction_vector = get_sigma_vector(robot_id, tether1_id, tether2_id, goal_sigma)
    curr_x = p.getLinkState(robot_id, 2)[0][0]
    curr_y = p.getLinkState(robot_id, 2)[0][1]
    # normalized_result = normalize_vector(direction_vector)

    new_position = [curr_x+(0.01)*direction_vector[0], curr_y+(0.01)*direction_vector[1]]

    return new_position

def new_position_forward_with_strain_1_tether(robot_id, tether_id):
    """
    Determines the new position the robot should move to to maintain its tether's goal strain based on 
    its heading and strain tether.
    """
    curr_x = p.getLinkState(robot_id, 2)[0][0]
    curr_y = p.getLinkState(robot_id, 2)[0][1]
    strain_vector = get_strain_vector(robot_id, tether_id)
    robot_heading = get_robot_heading(robot_id)
    gradient = go_home(robot_id, Home)
    # resulting_vector = [strain_weight*strain_vector[0]+heading_weight*robot_heading[0]+gradient_weight*gradient[0], 
    #                     strain_weight*strain_vector[1]+heading_weight*robot_heading[1]+gradient_weight*gradient[1]]
    resulting_vector = [strain_weight*strain_vector[0]+gradient_weight*gradient[0], 
                        strain_weight*strain_vector[1]+gradient_weight*gradient[1]]
    normalized_result = normalize_vector(resulting_vector)
    
    
    new_position = [curr_x+(0.03)*normalized_result[0], curr_y+(0.03)*normalized_result[1]]
    
    return new_position

def turn_around(robot_id):
    curr_pos = get_robot_heading(robot_id)
    scale_down = 2
    new_pos = [-scale_down*curr_pos[0], -scale_down*curr_pos[1]] 
    return new_pos

def go_home(robot_id, home):
    """
    Returns the global position pointing towards the origin,
    where the vector's magnitude increases the farther
    """
    curr_x = p.getLinkState(robot_id, 2)[0][0]
    curr_y = p.getLinkState(robot_id, 2)[0][1]
    home_x = home[0]
    home_y = home[1]
    distance = math.sqrt(((curr_x-home_x)**2)+((curr_y-home_y)**2))
    if distance == 0:
        distance = 1

    if distance >= 10*l_0:
        scale = 1
    elif distance >= 2*l_0:
        scale = 0.5
    else:
        scale = 0.1
    
    home_vector = [scale*(1/distance)*(home_x - curr_x), scale*(1/distance)*(home_y - curr_y)]
    # new_position = [curr_x + home_vector[0], curr_y + home_vector[1]]
    return home_vector

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

def normalize_vector(vec):
    """
    Takes any vector and returns a normalized vector
    """
    magnitude = get_magnitude(vec)
    normalized = [(1/magnitude)*vec[0],(1/magnitude)*vec[1]]
    return normalized

def get_magnitude(vector):
    """
    Returns the magnitude a two-dimensional vector
    """
    return math.sqrt((vector[0]**2) + (vector[1]**2))

def calculate_new_position(robot_id, tether1_id, tether2_id, goal_delta):
    """
    This is to calculate the resulting vector of a robot that has two tethers, a goal angle, 
    and a global gradient.
    """
    curr_x = p.getLinkState(robot_id, 2)[0][0]
    curr_y = p.getLinkState(robot_id, 2)[0][1]
    strain_vector1 = get_strain_vector(robot_id, tether1_id)
    strain_vector2 = get_strain_vector(robot_id, tether2_id)
    tot_strain_vector = [strain_vector1[0]+strain_vector2[0], strain_vector1[1]+strain_vector2[1]]
    robot_heading = get_robot_heading(robot_id)
    gradient = go_home(robot_id, Home)

    direction_vector = get_sigma_vector(robot_id, tether1_id, tether2_id, goal_delta)
    # curr_x = p.getLinkState(robot_id, 2)[0][0]
    # curr_y = p.getLinkState(robot_id, 2)[0][1]

    resulting_vector = [strain_weight*tot_strain_vector[0]+gradient_weight*gradient[0]+angle_weight*direction_vector[0], 
                        strain_weight*tot_strain_vector[1]+gradient_weight*gradient[1]+angle_weight*direction_vector[1]]
    normalized_result = normalize_vector(resulting_vector)
    
    
    position = [curr_x+(0.03)*normalized_result[0], curr_y+(0.03)*normalized_result[1]]

    return position

    
GRAVITYZ = -9.81  # m/s^2
N = 3 # number of agents to be created

radius = 0.1  # diameter of each robot in meters
mass = 1.0 # mass of each robot in kg
l_0 = 1   # unstretched/taut length of tether in meters
mu = 2.5  # friction coefficient between robots and plane
height = 0.005 # hieght of robot
Home = [-1, -1]
sensing_radius = radius * 4

# tether properties
goal_strain = 0.1
strain_weight = 6
heading_weight = 0
gradient_weight = 2
angle_weight = 5
repulsion_weight = 5

err_pos = 0.01 # positional error tolerance
err_delta = 5 # delta error tolerance

def main():
    p.connect(p.GUI) # connect to PyBullet GUI
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) # add pybullet_data to search path

    # set parameters
    p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI,0) # disable side bar windows in the GUI
    p.setGravity(0, 0, GRAVITYZ)
    p.setTimeStep(1./240.)

    # set initial object positions
    # initial_robot_positions = set_straight_line(N, l_0)
    
    initial_robot_positions = [[0,0,height],
                               [0,1, height],
                               [1,1, height]]
    # initial_robot_positions = [[1,1,height]]
    # initial_robot_positions = set_straight_line(N, l_0)
    
    # list of x-y current target positions for each agent (starts at their initial positions)
    target_pos = [initial_robot_positions[i][:2] for i in range(len(initial_robot_positions))]

    # each tile is a 1x1 meter square
    plane_id = p.loadURDF("plane.urdf")

    # a list of all of the agent objects created
    robot_ids = []

    # a list of tether objects
    tether_ids = []

    # populates the list of robot objects with robot objects
    for i in range(N):
        robot_ids.append(make_robot(radius, initial_robot_positions[i]))

    # applies friction/damping between robots and the plane
    for i in range(N):
        p.changeDynamics(robot_ids[i], -1, linearDamping=mu)

    # populates the list of tether objects with tether objects
    for i in range(N-1):
        tether_ids.append(make_tether(initial_robot_positions[i], initial_robot_positions[i+1], l_0, num_segments=1))

    # anchors all of the tethers to their respective robots
    for i in range(N-1):
        anchor_tether(tether_ids[i], robot_ids[i], robot_ids[i+1])
        
    runs = 0

    # print(get_sigma(robot_ids[1], tether_ids[0], tether_ids[1]))

    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)

        if runs%100 == 0:
            # calculate tether angle relative to each robot's heading
            sigma1 = get_sigma(robot_ids[1], tether_ids[0], tether_ids[1])
            # theta2 = get_theta(robot_ids[1], tether_ids[0])

            # display results in the GUI
            p.addUserDebugText(f"sigma = {sigma1:.2f} deg \n",
                                [0, 0.5, 0.5], textColorRGB=[0, 0, 0], lifeTime=1)
        

        # # calculate tether length and strain on every step
        # l = get_tether_length(tether_ids[0])
        # strain = (l - l_0) / l_0

        # # calculate tether angle relative to each robot's heading
        # sigma1 = get_sigma(robot_ids[1], tether_ids[0], tether_ids[1])
        # # theta2 = get_theta(robot_ids[1], tether_ids[0])

        # # calculates the direction the robot should move in to achieve the 
        # # goal delta
        # new_vector = new_position_for_sigma_goal(robot_ids[1], tether_ids[0], tether_ids[1], 270)

        # # display results in the GUI
        # p.addUserDebugText(f"tether length = {l:.2f} m\n tether strain = {strain:.2f}\n "
        #                     f"sigma = {sigma1:.2f} deg \n x_comp = {new_vector[0]:.2f} \n y_comp ={new_vector[1]:.2f} \n",
        #                     [0, 0.5, 0.5], textColorRGB=[0, 0, 0], lifeTime=1)
        

        if reached_target_position(robot_ids[1], target_pos[1][0], target_pos[1][1], err_pos):
            new_pos = calculate_new_position(robot_ids[1], tether_ids[0], tether_ids[1], 90)
            target_pos[1] = (new_pos[0], new_pos[1])
            move_robot(robot_ids[1], target_pos[1][0], target_pos[1][1], force=60)

        if reached_target_position(robot_ids[0], target_pos[0][0], target_pos[0][1], err_pos):
            new_pos = new_position_forward_with_strain_1_tether(robot_ids[0], tether_ids[0])
            target_pos[0] = (new_pos[0], new_pos[1])
            move_robot(robot_ids[0], target_pos[0][0], target_pos[0][1], force=60)

        if reached_target_position(robot_ids[2], target_pos[2][0], target_pos[2][1], err_pos):
            new_pos = new_position_forward_with_strain_1_tether(robot_ids[2], tether_ids[1])
            target_pos[2] = (new_pos[0], new_pos[1])
            move_robot(robot_ids[2], target_pos[2][0], target_pos[2][1], force=60)

        runs = runs + 1
        
        # if runs == 0:
        #     new_pos = turn_around(robot_ids[0])
        #     target_pos[0] = (new_pos[0], new_pos[1])
        #     move_robot(robot_ids[0], target_pos[0][0], target_pos[0][1], force=60)
        #     runs = 1
        # elif reached_target_position(robot_ids[0], target_pos[0][0], target_pos[0][1], err_pos):
        #     new_pos = new_position_forward_with_strain_1_tether(robot_ids[0], tether_ids[0])
        #     target_pos[0] = (new_pos[0], new_pos[1])
        #     move_robot(robot_ids[0], target_pos[0][0], target_pos[0][1], force=60)


        # if reached_target_position(robot_ids[1], target_pos[1][0], target_pos[1][1], err_pos):
        #     new_pos = new_position_for_sigma_goal(robot_ids[1], tether_ids[0], tether_ids[1], 150)
        #     target_pos[1] = (new_pos[0], new_pos[1])
        #     # print(get_sigma(robot_ids[1], tether_ids[0], tether_ids[1]))
        #     # print("\n")
        #     move_robot(robot_ids[1], target_pos[1][0], target_pos[1][1], force=60)




        # if reached_target_position(robot_ids[0], target_pos[0][0], target_pos[0][1], err_pos):
        #     new_pos = go_home(robot_ids[0], [-1,1])
        #     target_pos[0] = (new_pos[0], new_pos[1])
        #     move_robot(robot_ids[0], target_pos[0][0], target_pos[0][1], force=60)

        p.stepSimulation()

if __name__ == "__main__":
    main()
