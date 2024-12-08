import numpy as np

# My previous work that I don't know is relevant anymore
# - Wil
# 
# 
# class Pole:
#     def __init__(self, base, tip):
#         self.base = np.array(base)
#         self.tip = np.array(tip)

#         print(np.isclose(self.base, self.tip))
#         assert not all(np.isclose(self.base, self.tip)), "Both points on pole cannot overlap"

#         self.direction = (self.tip - self.base) / np.linalg.norm(self.tip - self.base)

# class RepulsionModel:
#     def __init__(self, pole, x_ef, k=1):
#         self.pole = pole
#         self.x_ef = np.array(x_ef)
#         self.k = k

#     @property
#     def repulsion_force(self):
#         # Vector from pole base to x_ef
#         w = self.x_ef - self.pole.base

#         # Closest point on the pole
#         projection_length = np.dot(w, self.pole.direction)
#         closest_point = self.pole.base + projection_length * self.pole.direction

#         # Direction of repulsion
#         repulsion_vector = self.x_ef - closest_point
#         distance = np.linalg.norm(repulsion_vector)

#         # Handle edge case: avoid division by zero
#         if distance == 0:
#             return np.zeros_like(repulsion_vector)

#         # Repulsion force (inverse square law, scaled by k)
#         return (repulsion_vector / distance**3) * self.k
    
# Hyperparameter
BETA = .25 # := proportion the magnitism matters (vs. the distance to goal)

class MotionPlanner:
    @staticmethod
    def get_random_point(point, sphere_radius):
        """ 
            Unfiromally samples a point in the sphere centered at `point` and of radius `sphere_radius`
            Credit: ChatGPT 4o 
        """
        
        # Step 1: Sample a random direction uniformly on the surface of a unit sphere
        direction = np.random.normal(0, 1, size=point.shape)
        direction /= np.linalg.norm(direction)  # Normalize to get a unit vector

        # Step 2: Sample a random distance within the sphere
        distance = sphere_radius * np.cbrt(np.random.uniform(0, 1))  # Scale uniformly

        # Step 3: Generate the random point
        random_point = point + direction * distance
        return random_point        
        # return [np.random.uniform(a, b), np.random.uniform(a, b), np.random.uniform(a, b)]

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


    def __init__(self, point, sphere_radius, num_samples):
        self.points = []
        for _ in range(num_samples):
            self.points.append(self.get_random_point(point, sphere_radius))

    def optimize(self, goal_point, pole_point_1, pole_point_2, beta):
        assert self.points, "self.points must be defined first"

        fin_arr = []
        for point in self.points:
            goal_point = np.array(goal_point)
            dist_to_goal = np.linalg.norm(goal_point - point)
            dist_to_pole = np.linalg.norm(self.closest_point_on_pole(point, pole_point_1, pole_point_2) - point)
            cost = self.cost(dist_to_goal, dist_to_pole, beta)
            # print("Point =", point)
            # print(f"Cost = {cost}")
            fin_arr.append(cost)
        return self.points[np.argmin(np.array(fin_arr))]
    
    @staticmethod
    def cost(dist_to_goal, dist_to_pole, beta):
        # print(f"DEBUG: dist_to_goal = {dist_to_goal}, dist_to_pole = {dist_to_pole}, beta = {beta}")
        return dist_to_goal + beta * 1/dist_to_pole**2
    
    # Debug functions
    def print_points(self):
        print("Points are:")
        for point in self.points:
            print(f"- {point}")

    # def print_losses(self, goal_point, pole_point_1, pole_point_2):
    #     self.get_all_losses(goal_point, pole_point_1, pole_point_2)
