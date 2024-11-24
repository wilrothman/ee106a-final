"""
    Underlying optimization model. See `../formulas.tex` for more information.
    @author Wil Louis Rothman
    November 2024 - December 2024
"""

import numpy as np
import scipy as sp
from scipy.stats import norm

from twists import *


sample_size_pole = 500 # the number of standard normal samples to take of the pole to calculate repulsion

class Pole:
    """
        Represents the pole. We are given two (unequal) points of the pole
    """
    def __init__(self, q_pole1, q_pole2):
        q_pole1, q_pole2 = list(q_pole1), list(q_pole2)
        assert len(q_pole1) == 3 and len(q_pole2) == 3, f"q_pole1 and q_pole2 must be 3D coordinates but instead got q1: {q_pole1} and q2: {q_pole2}"
        assert q_pole1 != q_pole2, f"Cannot define pole line as q1 = q2 = {q_pole1}"
        self.q1, self.q2 = np.array(q_pole1), np.array(q_pole2)
        self.v = self.q2 - self.q1

    def dist(self, p):
        """ Outputs distance between the line representing the pole and 3D coordinate vector p """
        w = p - self.q1
        return np.linalg.norm(np.cross(w, self.v)) / np.linalg.norm(self.v)
    
    def projection(self, p):
        """ Gets the projection (i.e. closest point) of p to the line """
        w = p - self.q1
        return (np.cross(w, self.v) / np.cross(self.v, self.v)) * self.v
    
    def get_sample_points(self, x_ef, sample_size_pole):
        """ Get samples of points for repulsion (Monte Carlo integration) """
        self.projection(x_ef)
        samples = norm.rvs(loc=0, scale=1, size=sample_size_pole) # Standard normal distribution
        
        p_vector = np.array([]) # the calculated sample points 
        for sample in samples:
            v_norm = np.linalg.norm(self.v)
            p_i = self.q1 + sample * v_norm
            p_vector = np.append(p_vector, p_i)
        return p_vector


class RepulsionModel:
    """
        Repulsion model as laid out under "Vision" notes on Notational Appendix.
        We model the pole as a line. We note that its diameter is 1 in.

        @param pole: the pole
        @param x_ef: the 3D coordinates of the ef
    """

    def __init__(self, pole, x_ef, sample_size_pole, k):
        x_ef = list(x_ef)
        assert len(x_ef) == 3, f"len_ef must be 3 instead got {x_ef}"
        assert type(pole) == Pole, f"type of pole must be Pole, got {type(pole)}."

        self.pole = pole 
        self.x_ef = x_ef
        self.sample_size_pole = sample_size_pole
        self.k = k
        r_vector, r_hat_vector = self.get_r_values()

        for i in range(len(r_vector)):
            r_i, r_hat_i = r_vector[i-1], r_hat_vector[i-1]

            self.repulsion_force = np.sum([(self.k / r_i **2) * r_hat_i])

        print("repulsive force =", self.repulsion_force)



    def get_r_values(self):
        """ 
            Calculate repulsion from the pole from the pole at a variety of points, 
            mostly centered around the closest point on the line to the EF

            @returns r_values, r_hat_values 
        """
        p_vector = self.pole.get_sample_points(self.x_ef, self.sample_size_pole)

        r_vector = np.array([])  # r_i = ||x_ef - p_i||
        r_hat_vector = np.array([]) # (r_hat)_i = (x_ef - p_i) / ||x_ef - p_i||
        for p_i in p_vector:
            r_vector = np.append(r_vector, np.linalg.norm(self.x_ef - p_i))
            r_hat_vector = np.append(r_hat_vector, (self.x_ef - p_i) / np.linalg.norm(self.x_ef - p_i))

        return r_vector, r_hat_vector


