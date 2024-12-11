#!/usr/bin/env python
import json
import numpy as np


class CacheHandler:
    # Underlying data structure: dict (goal_point -> waypoints)
    # Constructor: CacheHandler(optional filename)
    # Methods
    #   save():                     saves to cache file
    #   load():                     loads a cache file. Initiates a new one if one does not exist
    #   add(goal_point, waypoints): saves a new value to the cache
    #   get(goal_point):            gets the closest point to the goal point
    #

    def __init__(self, filename='cache.json'):
        #   Data structure: dict
        #   Key:  (start_point: tuple, goal_point: tuple, (pole_point_1: tuple, pole_point_2: tuple))
        #   Value: [waypoint_1, ..., waypoint_n]
        self.cache = {}
        self.filename = filename
        print("[CACHE] New handler instance created")

    def save(self):
        """Saves cache to file with stringified keys."""
        # Convert tuple keys to strings
        serializable_cache = {str(key): value for key, value in self.cache.items()}
        with open(self.filename, 'w') as file:
            json.dump(serializable_cache, file)
        print(f"[CACHE] Wrote cache to {self.filename}")


    def load(self):
        """Loads cache from file and converts keys back to tuples."""
        try:
            with open(self.filename, 'r') as file:
                loaded_cache = json.load(file)
                # Convert string keys back to tuples
                self.cache = {eval(key): value for key, value in loaded_cache.items()}
            print(f"[CACHE] Loaded cache from {self.filename}")
        except FileNotFoundError:
            print(f"[CACHE] Cache file {self.filename} not found. Initializing an empty cache.")
            self.cache = {}

    def add(self, goal_point, waypoints):
        # start, goal, pole_point_1, pole_point_2 = tuple(start), tuple(goal), tuple(pole_point_1), tuple(pole_point_2)
        goal_point = tuple(goal_point)
        waypoints = [tuple(waypoint) for waypoint in waypoints]

        if goal_point in self.cache.keys():
            print(f"[CACHE] Goal point {goal_point} already exists. Overwritting waypoints...")

        self.cache[goal_point] = waypoints
        print(F"[CACHE] Added {goal_point} -> {waypoints}")

    def get(self, goal_point):
        # Edge case. Hopefully shouldn't happen.
        if not self.cache:
            raise KeyError("[CACHE] Cache is empty. No keys to retrieve.")

        # Vectorize the problem
        candidate_goal_points = np.array(list(self.cache.keys()))
        # print("DEBUG: converted datatype keys -> list -> array")
        # print("DEBUG:", candidate_goal_points)

        # Finding minimum distance

        distances = list(map(
            lambda candidate: np.linalg.norm(candidate - goal_point),
            candidate_goal_points
        ))

        # print("DEBUG: Distances:", distances)
        closest_index = np.argmin(distances)
        # print("DEBUG: argmin =", closest_index)

        # Retrieval
        closest_key = tuple(candidate_goal_points[closest_index]) 

        print(f"[CACHE] Closest key to {goal_point}: {closest_key} with distance {distances[closest_index]}")
        return closest_key


    
