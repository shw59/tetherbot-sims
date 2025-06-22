"""
utils.py

This utilities file contains useful math functions for calculations in other class member functions.
"""

import numpy as np

GRAVITYZ = -9.81 # m/s^2

def normalize(vec):
    """
    Normalize a vector (must be a numpy array).
    """
    norm = np.linalg.norm(vec)
    return vec / norm if norm != 0 else vec

def magnitude_of_vector(vec):
    """
    Returns the magnitude of a 2D vector.
    """
    return np.sqrt((vec[0]**2)+(vec[1]**2))

def normal_force(mass):
    """
    Returns the normal force in Newtons of an object with specified mass (in kg).
    """
    return mass * abs(GRAVITYZ)
