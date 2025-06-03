"""
tether2.py

This file defines a simulation of two robots connected by a tether in PyBullet. The robots in this file are not 
constrained by prismatic joints and have wheels, allowing them to move freely in three dimensions.
"""

import pybullet as p
import pybullet_data
import numpy as np
import time
import math

def make_tether(length_0):
  """
  Return .obj text for a ribbon with length_0 segments.
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
  Create a URDF text describing a robot in the form of a disk with specified radius and/or length, mass, and color.
  """
  radius = diameter / 2

  # inertia of a solid cylinder about its own center
  ixx = iyy = (1/12) * mass * (3 * radius**2 + length**2)
  izz = 0.5 * mass * radius**2

  rgba = " ".join(map(str, color))

  # set position of heading-indicator block
  block_origin_x = radius / 2
  
  wheel_mass = 0.05 * mass
  wheel_radius = 0.20 * diameter
  wheel_offset_y = radius / 2


  urdf_text = f"""<?xml version="1.0"?>
  <robot name="disk">
    <link name="base_link">
      <visual>
        <origin xyz="0 0 {length/2 + wheel_radius}" rpy="0 0 0"/>
        <geometry>
          <cylinder length="{length}" radius="{radius}"/>
        </geometry>
        <material name="agent_color">
          <color rgba="{rgba}"/>
        </material>
      </visual>
      <collision>
        <origin xyz="0 0 {length / 2 + wheel_radius}" rpy="0 0 0"/>
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
      <origin xyz="{block_origin_x} 0 .005" rpy="0 0 0"/>
    </joint>

    <link name="heading_block">
      <visual>
        <origin xyz="{block_origin_x} 0 .005" rpy="0 0 0"/>
        <geometry>
          <box size="{radius} 0.01 0.01"/>
        </geometry>
        <material name="block_color"><color rgba="0 0 1 1"/></material>
      </visual>
      <collision>
        <origin xyz="{block_origin_x} 0 .005" rpy="0 0 0"/>
        <geometry>
          <box size="{radius} 0.01 0.01"/>
        </geometry>
      </collision>
    </link>

    <link name="right_wheel">
        <visual>
            <origin xyz="0 {wheel_offset_y} {wheel_radius}" rpy="1.5708 0 0"/>
            <geometry>
                <cylinder length="{length}" radius="{wheel_radius}"/>
            </geometry>
            <material name="wheel_gray">
                <color rgba="0 0 0 1"/>
            </material>
        </visual>
        <collision>
            <origin xyz="0 {wheel_offset_y} {wheel_radius}" rpy="1.5708 0 0"/>
            <geometry>
                <cylinder length="{length}" radius="{wheel_radius}"/>
            </geometry>
        </collision>
        <inertial>
            <origin xyz="0 {wheel_offset_y} {wheel_radius}" rpy="0 0 0"/>
            <mass value="{wheel_mass}"/>
            <inertia ixx="1e-4" ixy="0" ixz="0" iyy="1e-4" iyz="0" izz="1e-4"/>
        </inertial>
    </link>

    <joint name = "right_wheel_joint" type="continuous">
      <parent link="base_link"/>
      <child link="right_wheel"/>
      <origin xyz="0 {wheel_offset_y} {length}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
    </joint>

    <link name="left_wheel">
        <visual>
            <origin xyz="0 {-wheel_offset_y} {wheel_radius}" rpy="1.5708 0 0"/>
            <geometry>
                <cylinder length="{length}" radius="{wheel_radius}"/>
            </geometry>
            <material name="wheel_gray">
                <color rgba="0 0 0 1"/>
            </material>
        </visual>
        <collision>
            <origin xyz="0 {-wheel_offset_y} {wheel_radius}" rpy="1.5708 0 0"/>
            <geometry>
                <cylinder length="{length}" radius="{wheel_radius}"/>
            </geometry>
        </collision>
        <inertial>
            <origin xyz="0 {-wheel_offset_y} {wheel_radius}" rpy="0 0 0"/>
            <mass value="{wheel_mass}"/>
            <inertia ixx="1e-4" ixy="0" ixz="0" iyy="1e-4" iyz="0" izz="1e-4"/>
        </inertial>
    </link>

    <joint name = "left_wheel_joint" type="continuous">
      <parent link="base_link"/>
      <child link="left_wheel"/>
      <origin xyz="0 {-wheel_offset_y} {length}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
    </joint>

  </robot>
  """
  return urdf_text

def get_tether_length(tether_id):
  """
  Return the instantaneous length of the tether.
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
  Return heading vector of a robot based on the position of its heading block.
  """
  robot_pos = p.getBasePositionAndOrientation(robot_id)[0]
  head_pos = p.getLinkState(robot_id, 0)[0]
  heading = [head_pos[i] - robot_pos[i] for i in range(3)]

  return heading

def get_tether_heading(robot1_id, robot2_id):
  """
  Return the heading vector of the tether between two robots, 
  which is just the vector from the first robot to the second.
  """
  robot1_pos = p.getBasePositionAndOrientation(robot1_id)[0]
  robot2_pos = p.getBasePositionAndOrientation(robot2_id)[0]
  heading = [robot2_pos[i] - robot1_pos[i] for i in range(3)]

  return heading

def get_theta(robot_heading, tether_heading):
  """
  Return the angle between the robot's heading and the tether's heading (in degrees).
  The angle is computed using the dot product.
  """
  hx, hy = robot_heading[:2]
  tx, ty = tether_heading[:2]
  theta = math.atan2(hx*ty - hy*tx, hx*tx + hy*ty)

  return math.degrees(theta) % 360
  

GRAVITYZ = -9.81  # m/s^2

dmtr = 0.2  # diameter of each agent in meters
mass = 1.0 # mass of each agent in kg
l_0 = 1   # unstretched/taut length of tether in meters
mu = 5  # friction coefficient between agents and plane

debugging = False  # set to True to print face positions of tether

def main():
  p.connect(p.GUI) # connect to PyBullet GUI
  p.setAdditionalSearchPath(pybullet_data.getDataPath()) # add pybullet_data to search path

  # set parameters
  p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
  p.configureDebugVisualizer(p.COV_ENABLE_GUI,0) # disable side bar windows in the GUI
  p.setGravity(0, 0, GRAVITYZ)
  p.setTimeStep(1./240.)

  plane_orn = [0, 0, 0, 1]  # p.getQuaternionFromEuler([0, 0, 0])
  tether_pos = [0, 0, 0.1]  # base position of the tether
  robot1_pos = [0, -0.5, 0]  # position of the first agent
  robot2_pos = [0, 0.5, 0]  # position of the second agent

  # load objects
  tether_filename = "objects/tether.obj"
  open(tether_filename, "w").write(make_tether(length_0=l_0))

  robot_blue_filename = "objects/robot_blue.urdf"
  open(robot_blue_filename, "w").write(make_robot(diameter=dmtr, mass=mass, color=(0, 0, 1, 1)))

  robot_red_filename = "objects/robot_red.urdf"
  open(robot_red_filename, "w").write(make_robot(diameter=dmtr, mass=mass, color=(1, 0, 0, 1)))

  plane_id = p.loadURDF("plane.urdf")  # each tile is a 1x1 meter square
  robot1_id = p.loadURDF(robot_blue_filename, robot1_pos, plane_orn)
  robot2_id = p.loadURDF(robot_red_filename, robot2_pos, plane_orn)
  tether_id = p.loadSoftBody(tether_filename, 
                             basePosition = tether_pos, 
                             scale=1, 
                             mass=1., 
                             useNeoHookean=0, 
                             useBendingSprings=1,
                             useMassSpring=1, 
                             springElasticStiffness=40, 
                             springDampingStiffness=.1,
                             springDampingAllDirections = 1, 
                             useSelfCollision=0, 
                             frictionCoeff=0, 
                             useFaceContact=1)

  # set tether color and appearance
  p.changeVisualShape(tether_id, -1, rgbaColor=[1.0, 0.2, 0.58, 1.0], flags=p.VISUAL_SHAPE_DOUBLE_SIDED)

  # anchor the tethers to the robots
  num_verts, *_ = p.getMeshData(tether_id, -1, flags=p.MESH_DATA_SIMULATION_MESH)
  p.createSoftBodyAnchor(tether_id, 0, robot1_id, -1)
  p.createSoftBodyAnchor(tether_id, 1, robot1_id, -1)
  p.createSoftBodyAnchor(tether_id, num_verts-2, robot2_id, -1)
  p.createSoftBodyAnchor(tether_id, num_verts-1, robot2_id, -1)

  # apply non-zero friction to robots
  p.changeDynamics(robot1_id, -1, lateralFriction=mu)
  p.changeDynamics(robot2_id, -1, lateralFriction=mu)

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
    
    p.setJointMotorControl2(robot1_id, 2, p.POSITION_CONTROL, targetPosition=0.1, force=10)

    p.stepSimulation()

if __name__ == "__main__":
  main()