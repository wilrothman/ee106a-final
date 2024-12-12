#!/usr/bin/env python
import json
import numpy as np


import pickle

class CacheHandler:
    def __init__(self, filename='cache.pkl'):
        self.cache = {}
        self.filename = filename
        print("[CACHE] New handler instance created")

    def save(self):
        """Saves cache to a file using pickle."""
        with open(self.filename, 'wb') as file:
            pickle.dump(self.cache, file)
        print(f"[CACHE] Wrote cache to {self.filename}")

    def load(self):
        """Loads cache from a file using pickle."""
        try:
            with open(self.filename, 'rb') as file:
                self.cache = pickle.load(file)
            print(f"[CACHE] Loaded cache from {self.filename}")
        except FileNotFoundError:
            print(f"[CACHE] Cache file {self.filename} not found. Initializing an empty cache.")
            self.cache = {}

    def add(self, goal_point, trajectories):
        """Adds trajectories (list of plans) to the cache."""
        goal_point = tuple(goal_point)

        if goal_point in self.cache:
            print(f"[CACHE] Goal point {goal_point} already exists. Overwriting trajectories...")

        self.cache[goal_point] = trajectories
        print(f"[CACHE] Added {goal_point} -> {trajectories}")

    def get(self, goal_point):
        """Retrieves the closest key to the goal point."""
        if not self.cache:
            raise KeyError("[CACHE] Cache is empty. No keys to retrieve.")
        
        goal_point = tuple(goal_point)

        candidate_goal_points = np.array(list(self.cache.keys()))

        # Compute distances
        distances = np.linalg.norm(candidate_goal_points - goal_point, axis=1)
        closest_index = np.argmin(distances)
        closest_key = tuple(candidate_goal_points[closest_index])

        print(f"[CACHE] Closest key to {goal_point}: {closest_key} with distance {distances[closest_index]}")
        return self.cache[closest_key]



    
