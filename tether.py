"""
tether.py

This file defines the Tether class.
"""

import pybullet as p
import math
import os

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
        self.world_id = p
        self.length_0 = length_0

        dy = length_0 / num_segments  # length of every segment along the tether between each pair of vertices
        dx = 0.01  # half the width of the tether
        dz = 0.02 # height of tether (if applicable)

        lines = ["o tether"]
        # vertices
        for i in range(num_segments + 1):
            y = -dy * num_segments/2 + dy * i
            lines.append(f"v  { dx:.6f} {y:.6f} 0.000000")
            lines.append(f"v {-dx:.6f} {y:.6f} 0.000000")
            
            # # top
            # lines.append(f"v  { dx:.6f} {y:.6f} {dz:.6f}")
            # lines.append(f"v {-dx:.6f} {y:.6f} {dz:.6f}")
        # faces
        for i in range(num_segments):
            a, b, c, d = 2*i+1, 2*i+3, 2*i+2, 2*i+4
            lines += [f"f {a} {b} {c}", f"f {c} {b} {d}"]
                    
            # # bottom face
            # a, b, c, d = 4*i+1, 4*i+5, 4*i+2, 4*i+6
            # lines += [f"f {a} {b} {c}", f"f {c} {b} {d}"]

            # # top face
            # a, b, c, d = 4*i+3, 4*i+7, 4*i+4, 4*i+8
            # lines += [f"f {a} {b} {c}", f"f {c} {b} {d}"]

            # # right face
            # a, b, c, d = 4*i+1, 4*i+3, 4*i+5, 4*i+7
            # lines += [f"f {a} {b} {c}", f"f {c} {b} {d}"]

            # # left face
            # a, b, c, d = 4*i+2, 4*i+4, 4*i+6, 4*i+8
            # lines += [f"f {a} {b} {c}", f"f {c} {b} {d}"]

            # # front face (between top & bottom of first pair)
            # a, b, c, d = 4*i+1, 4*i+2, 4*i+3, 4*i+4
            # lines += [f"f {a} {b} {c}", f"f {c} {b} {d}"]

            # # back face (between top & bottom of next pair)
            # a, b, c, d = 4*i+5, 4*i+6, 4*i+7, 4*i+8
            # lines += [f"f {a} {b} {c}", f"f {c} {b} {d}"]

        tether_filename = f"objects/tether.obj"
        open(tether_filename, "w").write("\n".join(lines))

        # properties of paracord 550 (type 3)
        cross_area = math.pi * (diameter / 2)**2
        stiffness = youngs_modulus * cross_area / length_0

        self.id = self.world_id.loadSoftBody(os.path.abspath(tether_filename), 
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
        
        self.world_id.changeVisualShape(self.id, -1, rgbaColor=[1.0, 0.5, 0, 1.0], flags=p.VISUAL_SHAPE_DOUBLE_SIDED)
    
    def get_strain(self):
        """
        Return the current strain of the tether object.
        """
        n_verts, verts, *_ = self.get_verts()

        length = 0.0
        for i in range(0, n_verts-3, 2): # sum of distances between each pair of vertices on the tether mesh
            p1 = [(verts[i][k]   + verts[i+1][k]) / 2.0 for k in range(3)] # midpoints between the vertices
            p2 = [(verts[i+2][k] + verts[i+3][k]) / 2.0 for k in range(3)]
            length += math.dist(p1, p2)

        return (length - self.length_0) / self.length_0

        # num_sections = n_verts // 4
        # centers = []

        # for i in range(num_sections):
        #     base = 4 * i
        #     # bottom vertices only: BR (0) & BL (1)
        #     cx = (verts[base][0] + verts[base+1][0]) / 2.0
        #     cy = (verts[base][1] + verts[base+1][1]) / 2.0
        #     cz = (verts[base][2] + verts[base+1][2]) / 2.0
        #     centers.append((cx, cy, cz))

        # length = 0.0
        # for i in range(num_sections - 1):
        #     length += math.dist(centers[i], centers[i+1])

        # return (length - self.length_0) / self.length_0
    
    def get_verts(self):
        """
        Return the tether's number of vertices and the mesh vertices themselves as a tuple (n_verts, verts).
        """
        n_verts, verts, *_ = self.world_id.getMeshData(self.id, -1, flags=p.MESH_DATA_SIMULATION_MESH)

        return n_verts, verts
    