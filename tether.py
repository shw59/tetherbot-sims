"""
tether.py

This file defines the Tether class.
"""

import pybullet as p
import math

class Tether:
    def __init__(self, position_0, length_0, orientation_0, num_segments=10, mass=1.0, friction_coeff=0):
        """
        Initializes the tether object and its length_0 (unstretched length) and id attributes.
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

        self.id = p.loadSoftBody(tether_filename, 
                                basePosition = position_0, 
                                baseOrientation = orientation_0,
                                scale=1, 
                                mass=mass, 
                                useNeoHookean=0, 
                                useBendingSprings=1,
                                useMassSpring=1, 
                                springElasticStiffness=30, 
                                springDampingStiffness=.1,
                                springDampingAllDirections=1, 
                                useSelfCollision=0, 
                                frictionCoeff=friction_coeff, 
                                useFaceContact=1)
        
        p.changeVisualShape(self.id, -1, rgbaColor=[1.0, 0.2, 0.58, 1.0], flags=p.VISUAL_SHAPE_DOUBLE_SIDED)

    def length(self):
        """
        Return the current length of the tether.
        """
        n_verts, verts, *_ = p.getMeshData(self.id, -1, flags=p.MESH_DATA_SIMULATION_MESH)

        length = 0.0
        for i in range(0, n_verts-3, 2): # sum of distances between each pair of vertices on the tether mesh
            p1 = [(verts[i][k]   + verts[i+1][k]) / 2.0 for k in range(3)] # midpoints between the vertices
            p2 = [(verts[i+2][k] + verts[i+3][k]) / 2.0 for k in range(3)]
            length += math.dist(p1, p2)

        return length
    
    def strain(self):
        """
        Return the current strain of the tether object based on its current length.
        """
        return (self.length() - self.length_0) / self.length_0
    
    def get_verts(self):
        """
        Return the tether's number of vertices and the mesh vertices themselves as a tuple (n_verts, verts).
        """
        n_verts, verts, *_ = p.getMeshData(self.id, -1, flags=p.MESH_DATA_SIMULATION_MESH)

        return n_verts, verts
    