from scipy.interpolate import CubicSpline
import numpy as np


class MotionPlanner:
    def __init__(self, point, sphere_radius, num_samples):
        self.points = []
        for _ in range(num_samples):
            self.points.append(self.get_random_point(point, sphere_radius))

    @staticmethod
    def get_random_point(point, sphere_radius):
        """
        Uniformly samples a point in the sphere centered at `point` and of radius `sphere_radius`.
        """
        direction = np.random.normal(0, 1, size=point.shape)
        direction /= np.linalg.norm(direction)  # Normalize to get a unit vector
        distance = sphere_radius * np.cbrt(np.random.uniform(0, 1))  # Scale uniformly
        return point + direction * distance

    @staticmethod
    def closest_point_on_pole(point, pole_point_1, pole_point_2):
        point = np.array(point)
        pole_point_1, pole_point_2 = np.array(pole_point_1), np.array(pole_point_2)
        pole_to_point_vector = point - pole_point_1
        pole_direction = (pole_point_2 - pole_point_1) / np.linalg.norm(pole_point_2 - pole_point_1)
        projection_length = np.dot(pole_to_point_vector, pole_direction)
        pole_length = np.linalg.norm(pole_point_2 - pole_point_1)
        projection_length = max(0, min(projection_length, pole_length))
        closest_point = pole_point_1 + projection_length * pole_direction
        return closest_point

    @staticmethod
    def cost(dist_to_goal, dist_to_pole, beta):
        """
        Calculates a cost function based on:
        - Distance to the goal (minimize).
        - Repulsion from the pole (maximize distance to pole).
        """
        repulsion_penalty = beta / dist_to_pole**2 if dist_to_pole > 0 else float('inf')
        return dist_to_goal + repulsion_penalty

    def optimize(self, current_point, goal_point, pole_point_1, pole_point_2, beta):
        """
        Finds the best candidate point that minimizes the cost function.
        """
        assert self.points, "self.points must be defined first"

        best_point = None
        lowest_cost = float('inf')
        for point in self.points:
            dist_to_goal = np.linalg.norm(goal_point - point)
            dist_to_pole = np.linalg.norm(self.closest_point_on_pole(point, pole_point_1, pole_point_2) - point)
            cost = self.cost(dist_to_goal, dist_to_pole, beta)
            if cost < lowest_cost:
                lowest_cost = cost
                best_point = point
        return best_point

    @staticmethod
    def smooth_trajectory(points):
        """
        Smooths a list of waypoints for a trajectory using cubic spline interpolation.
        """
        points = np.array(points)
        time = np.linspace(0, 1, len(points))
        cs = CubicSpline(time, points, axis=0)
        smooth_points = cs(np.linspace(0, 1, len(points) * 10))  # Add more points for smoothness
        return smooth_points
