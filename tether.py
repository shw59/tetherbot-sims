"""
tether.py

This file defines the Tether class.
"""

import pybullet as p
import math

class Tether:
    label = "tether"
    def __init__(self, position_0, length_0, orientation_0, num_segments, mass, youngs_modulus, diameter, mu):
        """
        Initializes the tether object and its length_0 (unstretched length) and id attributes.

        position_0: Initial position of the tether in the format [x, y, z]
        length_0: Nominal length of the tether in meters
        orientation_0: A quaternion in the form of a 4-element list [w, x, y, z]
        num_segments: number of flexible spring-like segments the tether is spliced into
        stiffness: stiffness of the tether in N/m
        mass: mass of the tether in kg
        mu: contact friction coefficient of the tether
        """
        self.length_0 = length_0

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

        tether_filename = f"objects/tether.obj"
        open(tether_filename, "w").write("\n".join(lines))

        # properties of paracord type I
        # cross_area = math.pi * (diameter / 2)**2
        # stiffness = youngs_modulus * cross_area / length_0

        stiffness = 2500 # should figure out what it is for a paracord 550 or something stiffer

        self.id = p.loadSoftBody(tether_filename, 
                                basePosition = position_0, 
                                baseOrientation = orientation_0,
                                scale=1, 
                                mass=mass, 
                                useNeoHookean=0, 
                                useBendingSprings=1,
                                useMassSpring=1, 
                                springElasticStiffness=stiffness, 
                                springDampingStiffness=1.5,
                                springDampingAllDirections=10,
                                useSelfCollision=0, 
                                frictionCoeff=mu, 
                                useFaceContact=1)
        
        p.changeVisualShape(self.id, -1, rgbaColor=[1.0, 0.2, 0.58, 1.0], flags=p.VISUAL_SHAPE_DOUBLE_SIDED)
    
    def get_strain(self):
        """
        Return the current strain of the tether object.
        """
        n_verts, verts, *_ = p.getMeshData(self.id, -1, flags=p.MESH_DATA_SIMULATION_MESH)

        length = 0.0
        for i in range(0, n_verts-3, 2): # sum of distances between each pair of vertices on the tether mesh
            p1 = [(verts[i][k]   + verts[i+1][k]) / 2.0 for k in range(3)] # midpoints between the vertices
            p2 = [(verts[i+2][k] + verts[i+3][k]) / 2.0 for k in range(3)]
            length += math.dist(p1, p2)

        return (length - self.length_0) / self.length_0
    
    def get_verts(self):
        """
        Return the tether's number of vertices and the mesh vertices themselves as a tuple (n_verts, verts).
        """
        n_verts, verts, *_ = p.getMeshData(self.id, -1, flags=p.MESH_DATA_SIMULATION_MESH)

        return n_verts, verts
    