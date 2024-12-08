import numpy as np

class Pole:
    def __init__(self, base, tip):
        self.base = np.array(base)
        self.tip = np.array(tip)

        print(np.isclose(self.base, self.tip))
        assert not all(np.isclose(self.base, self.tip)), "Both points on pole cannot overlap"

        self.direction = (self.tip - self.base) / np.linalg.norm(self.tip - self.base)

class RepulsionModel:
    def __init__(self, pole, x_ef, k=1):
        self.pole = pole
        self.x_ef = np.array(x_ef)
        self.k = k

    @property
    def repulsion_force(self):
        # Vector from pole base to x_ef
        w = self.x_ef - self.pole.base

        # Closest point on the pole
        projection_length = np.dot(w, self.pole.direction)
        closest_point = self.pole.base + projection_length * self.pole.direction

        # Direction of repulsion
        repulsion_vector = self.x_ef - closest_point
        distance = np.linalg.norm(repulsion_vector)

        # Handle edge case: avoid division by zero
        if distance == 0:
            return np.zeros_like(repulsion_vector)

        # Repulsion force (inverse square law, scaled by k)
        return (repulsion_vector / distance**3) * self.k
    

class MotionPlanner:
    def __init__(self, point_range, num_samples):
        self.points = []
        for _ in range(num_samples):
            self.points.append(self.get_random_point(point_range[0], point_range[1]))

    @staticmethod
    def get_random_point(a, b):
        return [np.random.uniform(a, b), np.random.uniform(a, b), np.random.uniform(a, b)]

    def optimize(self, goal_point, pole_point, beta):
        fin_arr = []
        for point in self.points:
            goal_point, pole_point, point = np.array(goal_point), np.array(pole_point). np.array(point)
            dist_to_goal = np.norm(goal_point - point)
            dist_to_pole = np.norm(pole_point - point)
            cost = cost(dist_to_goal, dist_to_pole, beta)
            fin_arr.append(cost)
        return self.points[np.argmin(np.array(fin_arr))]
    
    def cost(dist_to_goal, dist_to_pole, beta):
        return dist_to_goal + beta * 1/dist_to_pole**2