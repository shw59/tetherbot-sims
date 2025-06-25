"""
utils.py

This utilities file contains useful math functions for calculations in other class member functions.
"""

import numpy as np
from scipy.optimize import least_squares

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

def get_collective_radius(points):
    """
    Finds the best fit circle for a group of agent positions and returns the radius of that circle in meters.
    """
    points = np.array(points)
    x = points[:, 0]
    y = points[:, 1]

    # initial guess
    x_m = np.mean(x)
    y_m = np.mean(y)
    r_initial = np.mean(np.sqrt((x - x_m)**2 + (y - y_m)**2))
    initial_guess = [x_m, y_m, r_initial]

    def residuals(params):
        xc, yc, r = params
        return np.sqrt((x - xc)**2 + (y - yc)**2) - r

    result = least_squares(residuals, initial_guess)

    _, _, r = result.x
    return r
