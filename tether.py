"""
tether.py

This file defines a PyBullet simulation of two controllable robots connected by a flexible tethe.
"""

import pybullet as p
import pybullet_data
import math

def make_tether(length_0):
    """
    Return .obj text for a tether (essentially a thin cloth) with length_0 segments.
    """
    dy = 1.0  # length of every segment along the tether between each pair of vertices
    dx = 0.01 # half the width of the tether
    lines = ["o tether"]
    # vertices
    for i in range(length_0 + 1):
        y = -dy * length_0/2 + dy * i
        lines.append(f"v  { dx:.6f} {y:.6f} 0.000000")
        lines.append(f"v {-dx:.6f} {y:.6f} 0.000000")
    # faces
    for i in range(length_0):
        a, b, c, d = 2*i+1, 2*i+3, 2*i+2, 2*i+4
        lines += [f"f {a} {b} {c}", f"f {c} {b} {d}"]
    return "\n".join(lines)
    
def make_robot(diameter, length=.01, mass=1.0, color=(0, 0.5, 1, 1), joint_type="prismatic"):
    """
    Create URDF text describing a cylindrical robot with specified radius and/or length, mass, and color.
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

        <joint name="y_to_world" type="{joint_type}">
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

        <joint name="x_to_y" type="{joint_type}">
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
    return urdf_text

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

def get_tether_heading(robot1_id, robot2_id):
    """
    Return the heading vector [x, y] of the tether between two robots, 
    which is just the vector from the first robot to the second.
    """
    robot1_pos = p.getLinkState(robot1_id, 2)[0]
    robot2_pos = p.getLinkState(robot2_id, 2)[0]
    heading = [robot2_pos[i] - robot1_pos[i] for i in range(3)]

    return heading[:2]

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

    print(f"base_heading={math.degrees(base_heading):.2f}, "
          f"desired_heading={math.degrees(desired_heading):.2f}, "
          f"rotation={math.degrees(rotation):.2f}")

    joint_indices = [1, 0, 2] # [x-direction, y-direction, rotation/heading]
    p.setJointMotorControlArray(robot_id, joint_indices, p.POSITION_CONTROL,
                                targetPositions=[x_move, y_move, rotation], forces=[force]*3)

    while p.getLinkState(robot_id, 2)[0][0] < x - err or p.getLinkState(robot_id, 2)[0][0] > x + err or \
          p.getLinkState(robot_id, 2)[0][1] < y - err or p.getLinkState(robot_id, 2)[0][1] > y + err:
        p.getCameraImage(320,200)
        p.stepSimulation()

    
GRAVITYZ = -9.81  # m/s^2

dmtr = 0.2  # diameter of each robot in meters
mass = 1.0 # mass of each robot in kg
l_0 = 1   # unstretched/taut length of tether in meters
mu = 2.5  # friction coefficient between robots and plane

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
    tether_pos = [0, 0, 0]  # base position of the tether
    robot1_pos = [0, -0.5, 0.005]  # base position of the first robot
    robot2_pos = [0, 0.5, 0.005]  # base position of the second robot

    # load objects
    tether_filename = "objects/tether.obj"
    open(tether_filename, "w").write(make_tether(length_0=l_0))

    robot_blue_filename = "objects/robot_blue.urdf"
    open(robot_blue_filename, "w").write(make_robot(diameter=dmtr, mass=mass, color=(0, 0, 1, 1)))

    robot_red_filename = "objects/robot_red.urdf"
    open(robot_red_filename, "w").write(make_robot(diameter=dmtr, mass=mass, color=(1, 0, 0, 1)))

    plane_id = p.loadURDF("plane.urdf")  # each tile is a 1x1 meter square
    robot1_id = p.loadURDF(robot_blue_filename, robot1_pos)
    robot2_id = p.loadURDF(robot_red_filename, robot2_pos)
    tether_id = p.loadSoftBody(tether_filename, 
                               basePosition = tether_pos, 
                               scale=1, 
                               mass=1., 
                               useNeoHookean=0, 
                               useBendingSprings=1,
                               useMassSpring=1, 
                               springElasticStiffness=40, 
                               springDampingStiffness=.1,
                               springDampingAllDirections=1, 
                               useSelfCollision=0, 
                               frictionCoeff=0, 
                               useFaceContact=1)

    # set tether color and appearance
    p.changeVisualShape(tether_id, -1, rgbaColor=[1.0, 0.2, 0.58, 1.0], flags=p.VISUAL_SHAPE_DOUBLE_SIDED)

    # anchor the tethers to the robots
    num_verts, *_ = p.getMeshData(tether_id, -1, flags=p.MESH_DATA_SIMULATION_MESH)
    # p.createSoftBodyAnchor(tether_id, 0, robot1_id, 1)
    # p.createSoftBodyAnchor(tether_id, 1, robot1_id, 1)
    # p.createSoftBodyAnchor(tether_id, num_verts-2, robot2_id, 1)
    # p.createSoftBodyAnchor(tether_id, num_verts-1, robot2_id, 1)

    # apply friction/damping between robots and the plane
    p.changeDynamics(robot1_id, -1, linearDamping=mu)
    p.changeDynamics(robot2_id, -1, linearDamping=mu)

    # debugging prints to get vertex positions on the tether
    if debugging:
        data = p.getMeshData(tether_id, -1, flags=p.MESH_DATA_SIMULATION_MESH)
        print("--------------")
        print("data=",data)
        print(data[0])
        print(data[1])
        text_uid = []
        for i in range(data[0]):
            pos = data[1][i]
            uid = p.addUserDebugText(str(i), pos, textColorRGB=[1,1,1])
            text_uid.append(uid)
    
    move_robot(robot1_id, 0, -1, 5)
    move_robot(robot1_id, -1, -1, 5)
    move_robot(robot1_id, -1, 0, 5)
    move_robot(robot1_id, 0, 0, 5)
    move_robot(robot1_id, 1, 1, 5)
    move_robot(robot1_id, 1, 0, 5)
    move_robot(robot1_id, 0, 1, 5)

    # main simulation loop
    while p.isConnected():
        p.getCameraImage(320,200)

        # calculate tether length and strain on every step
        l = get_tether_length(tether_id)
        strain = (l - l_0) / l_0

        # calculate tether angle relative to each robot's heading
        robot1_heading = get_robot_heading(robot1_id)
        robot2_heading = get_robot_heading(robot2_id)
        tether_heading1_2 = get_tether_heading(robot1_id, robot2_id)
        tether_heading2_1 = get_tether_heading(robot2_id, robot1_id)
        theta1 = get_theta(robot1_heading, tether_heading1_2)
        theta2 = get_theta(robot2_heading, tether_heading2_1)

        # display results in the GUI
        p.addUserDebugText(f"tether length = {l:.2f} m\n tether strain = {strain:.2f}\n "
                            f"theta_blue = {theta1:.2f} deg\n theta_red = {theta2:.2f} deg",
                            [0, 0.5, 0.5], textColorRGB=[0, 0, 0], lifeTime=1)

        p.stepSimulation()

if __name__ == "__main__":
  main()