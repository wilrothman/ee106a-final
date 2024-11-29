"""
    Work in progress!!
    @author Wil Louis Rothman
    November 2024 - December 2024
"""

import numpy as np
from scipy.optimize import minimize
from model import *

# Define the repulsion model and Sawyer SDK
class SawyerArm:
    def __init__(self, initial_twists):
        self.twists = np.array(initial_twists)

    def forward_kinematics(self, twists):
        # Replace with actual SDK call to compute EF position
        # Example: rospy or moveit integration for Sawyer arm
        return compute_ef_position_from_twists(twists)

# Objective function: Minimize repulsion
def objective(twists, sawyer_arm, repulsion_model):
    # Calculate the end-effector position
    ef_position = sawyer_arm.forward_kinematics(twists)

    # Compute the repulsion force at this position
    force = repulsion_model.repulsion_force(ef_position)

    # Objective: Minimize the magnitude of the repulsion force
    return np.linalg.norm(force)**2

# Joint limits for Sawyer (example limits; adjust to real specs)
joint_limits = [
    (-3.0, 3.0),  # Joint 1
    (-3.0, 3.0),  # Joint 2
    (-3.0, 3.0),  # Joint 3
    (-3.0, 3.0),  # Joint 4
    (-3.0, 3.0),  # Joint 5
    (-3.0, 3.0),  # Joint 6
    (-3.0, 3.0),  # Joint 7
]

# Initialize Sawyer arm and repulsion model
initial_twists = [0, 0, 0, 0, 0, 0, 0]  # Initial joint configuration
sawyer_arm = SawyerArm(initial_twists)
repulsion_model = RepulsionModel(pole, [0, 0, 0], k=1)

# Perform optimization
result = minimize(
    fun=objective,
    x0=initial_twists,
    args=(sawyer_arm, repulsion_model),
    bounds=joint_limits,
    method='SLSQP'  # Sequential Least Squares Programming
)

# Output the optimized twists
print("Optimized Twists:", result.x)
print("Minimum Repulsion Force:", result.fun)
