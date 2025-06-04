"""
tether.py

This file defines a PyBullet simulation of two controllable robots connected by a flexible tether.
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

    tether_filename = f"objects/{name}.obj"
    open(tether_filename, "w").write("\n".join(lines))

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
    
def make_robot(name, diameter, position, length=.01, mass=1.0, color=(0, 0.5, 1, 1)):
    """
    Returns the id of a cylindrical robot object with specified radius and/or length, mass, and color.
    """
    radius = diameter / 2

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

    robot_blue_filename = f"objects/{name}.urdf"
    open(robot_blue_filename, "w").write(urdf_text)

    return p.loadURDF(robot_blue_filename, position)

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

def get_theta(robot_heading, tether_heading):
    """
    Return the angle between the robot's heading and the tether's heading (in degrees).
    The angle is computed using the dot product.
    """
    hx, hy = robot_heading
    tx, ty = tether_heading
    theta = math.atan2(hx*ty - hy*tx, hx*tx + hy*ty)

    return math.degrees(theta) % 360

def move_robot(robot_id, x, y, force=10, err=0.01):
    """
    Move the robot to a specified position (x, y) with an optionally specified force and positional error tolerance.
    """
    # amount to move (relative to base position)
    x_move = x - p.getBasePositionAndOrientation(robot_id)[0][0]
    y_move = y - p.getBasePositionAndOrientation(robot_id)[0][1]

    # calculate rotation to face direction of movement
    base_heading = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(robot_id)[1])[2] # starting heading
    y_heading = y - p.getLinkState(robot_id, 2)[0][1] # y_move relative to current position
    x_heading = x - p.getLinkState(robot_id, 2)[0][0] # x_move relative to current position
    desired_heading = math.atan2(y_heading, x_heading)
    rotation = (desired_heading - base_heading + math.pi) % (2 * math.pi) - math.pi # smallest signed angle difference

    # print(f"base_heading={math.degrees(base_heading):.2f}, "
    #       f"desired_heading={math.degrees(desired_heading):.2f}, "
    #       f"rotation={math.degrees(rotation):.2f}")

    joint_indices = [1, 0, 2] # [x-direction, y-direction, rotation/heading]
    p.setJointMotorControlArray(robot_id, joint_indices, p.POSITION_CONTROL,
                                targetPositions=[x_move, y_move, rotation], forces=[force]*3)

    while p.getLinkState(robot_id, 2)[0][0] < x - err or p.getLinkState(robot_id, 2)[0][0] > x + err or \
          p.getLinkState(robot_id, 2)[0][1] < y - err or p.getLinkState(robot_id, 2)[0][1] > y + err:
        p.getCameraImage(320,200)
        p.stepSimulation()

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
    
    # print(positions)

    return positions

    
GRAVITYZ = -9.81  # m/s^2
N = 2 # number of agents to be created

dmtr = 0.2  # diameter of each robot in meters
mass = 1.0 # mass of each robot in kg
l_0 = 1   # unstretched/taut length of tether in meters
mu = 2.5  # friction coefficient between robots and plane
height = 0.005 # hieght of robot

debugging = False  # set to True to print vertex positions of tether

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

    # print(initial_robot_positions)
    # initial_robot_positions.append([1 ,0, height])
    initial_robot_positions = [[0,0,height],
                               [1, 0, height]]

    # each tile is a 1x1 meter square
    plane_id = p.loadURDF("plane.urdf")

    # a list of all of the agent objects created
    robot_ids = []

    # a list of tether objects
    tether_ids = []

    # populates the list of robot objects with robot objects
    for i in range(N):
        robot_ids.append(make_robot(f"robot{i+1}", dmtr, initial_robot_positions[i]))

    # applies friction/damping between robots and the plane
    for i in range(N):
        p.changeDynamics(robot_ids[i], -1, linearDamping=mu)

    # populates the list of tether objects with tether objects
    for i in range(N-1):
        tether_ids.append(make_tether(f"tether{i+1}", initial_robot_positions[i], initial_robot_positions[i+1], l_0, num_segments=1))

    # anchors all of the tethers to their respective robots
    for i in range(N-1):
        anchor_tether(tether_ids[i], robot_ids[i], robot_ids[i+1])

    # debugging prints to get vertex positions on the tether
    if debugging:
        data = p.getMeshData(tether_ids[0], -1, flags=p.MESH_DATA_SIMULATION_MESH)
        print("--------------")
        print("data=",data)
        print(data[0])
        print(data[1])
        text_uid = []
        for i in range(data[0]):
            pos = data[1][i]
            uid = p.addUserDebugText(str(i), pos, textColorRGB=[1,1,1])
            text_uid.append(uid)
    
    # move robots
    move_robot(robot_ids[1], 1, 1, 30)
    move_robot(robot_ids[1], -1, 1, 30)
    move_robot(robot_ids[1], -1, -1, 30)
    move_robot(robot_ids[1], 1, -1, 30)

    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)

        # calculate tether length and strain on every step
        l = get_tether_length(tether_ids[0])
        strain = (l - l_0) / l_0

        # calculate tether angle relative to each robot's heading
        robot1_heading = get_robot_heading(robot_ids[0])
        robot2_heading = get_robot_heading(robot_ids[1])
        tether_heading1 = get_tether_heading(robot_ids[0], tether_ids[0])
        tether_heading2 = get_tether_heading(robot_ids[1], tether_ids[0])
        theta1 = get_theta(robot1_heading, tether_heading1)
        theta2 = get_theta(robot2_heading, tether_heading2)

        # display results in the GUI
        p.addUserDebugText(f"tether length = {l:.2f} m\n tether strain = {strain:.2f}\n "
                            f"theta_blue = {theta1:.2f} deg\n theta_red = {theta2:.2f} deg",
                            [0, 0.5, 0.5], textColorRGB=[0, 0, 0], lifeTime=1)

        p.stepSimulation()

if __name__ == "__main__":
  main()