import numpy as np

class Pole:
    def __init__(self, base, tip):
        self.base = np.array(base)
        self.tip = np.array(tip)

        assert not any(np.isclose(self.base, self.tip)), "Both points on pole cannot overlap"

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
