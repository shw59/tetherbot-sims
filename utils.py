"""
utils.py

This utilities file contains useful math functions for calculations in other class member functions.
"""

import numpy as np

def normalize(vec):
    """
    Normalize a vector (must be a numpy array).
    """
    norm = np.linalg.norm(vec)
    if norm == 0:
        return vec
    return vec / norm

def magnitude_of_vector(vec):
    """
    Returns the magnitude of a 2D vector.
    """
    return np.sqrt((vec[0]**2)+(vec[1]**2))

