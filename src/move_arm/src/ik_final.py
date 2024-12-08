#!/usr/bin/env python
import rospy
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped
import numpy as np
from model import MotionPlanner


def add_collision_object():
    """
    Adds the pole as a collision object in the MoveIt! planning scene.
    """
    scene = PlanningSceneInterface()
    rospy.sleep(2)  # Allow the scene to initialize

    pole_size = [0.1, 0.1, 2.0]  # Width, Depth, Height
    pole_pose = PoseStamped()
    pole_pose.header.frame_id = "base"
    pole_pose.pose.position.x = 0.816
    pole_pose.pose.position.y = -0.500
    pole_pose.pose.position.z = 1.0  # Center of the pole
    pole_pose.pose.orientation.w = 1.0

    scene.add_box("pole", pole_pose, size=pole_size)
    rospy.sleep(2)


def interpolate_joint_trajectory(waypoints, steps=10):
    """
    Interpolates between joint-space waypoints for a smooth trajectory.
    """
    waypoints = np.array(waypoints)
    interpolated = []
    for i in range(len(waypoints) - 1):
        for t in np.linspace(0, 1, steps):
            interpolated.append((1 - t) * waypoints[i] + t * waypoints[i + 1])
    return interpolated


def main():
    rospy.init_node('service_query')

    group = MoveGroupCommander("right_arm")
    group.set_planning_time(10.0)

    # Add collision object
    add_collision_object()

    # Initial setup
    START_EF_POSITION = (0.816, 0.161, 0.642)
    GOAL = (0.816, -0.500, 0.642)
    NUM_SAMPLES = 1000
    CUSTOM_BETA = 0.25
    MAX_DIST_POLE = 1.0
    POLE_POINT_1 = (0.816, -0.500, 0)
    POLE_POINT_2 = (0.816, -0.500, 2)
    NUM_STEPS = 10
    STEP_SIZE = MAX_DIST_POLE / NUM_STEPS

    ef_posn = np.array(START_EF_POSITION)  # Current end-effector position
    planned_points = [START_EF_POSITION]  # Start trajectory with the initial position
    current_joints = group.get_current_joint_values()  # Get initial joint positions

    # Iteratively expand the sphere and optimize the path
    for step in range(NUM_STEPS):
        sphere_radius = (step + 1) * STEP_SIZE
        print(f"Sphere size (step {step}): {sphere_radius}")

        # Create a motion planner for the current sphere
        motion_planner = MotionPlanner(ef_posn, sphere_radius, NUM_SAMPLES)

        # Optimize the next point
        optimized = motion_planner.optimize(ef_posn, GOAL, POLE_POINT_1, POLE_POINT_2, CUSTOM_BETA)
        print(f"Optimized (step {step}): {optimized}")

        # Update the current end-effector position
        ef_posn = optimized
        planned_points.append(optimized)

    # Smooth the trajectory
    smooth_trajectory = motion_planner.smooth_trajectory(planned_points)

    # Create joint-space waypoints
    joint_waypoints = []
    for pose in smooth_trajectory:
        group.set_pose_target(pose)
        plan = group.plan()

        if plan and len(plan.joint_trajectory.points) > 0:
            joint_waypoints.append(plan.joint_trajectory.points[-1].positions)

    # Interpolate joint trajectory for smooth movement
    joint_trajectory = interpolate_joint_trajectory(joint_waypoints)

    # Execute the joint trajectory
    for joints in joint_trajectory:
        group.set_joint_value_target(joints)
        plan = group.plan()
        if plan and len(plan.joint_trajectory.points) > 0:
            group.execute(plan, wait=True)

    print("Execution completed.")


if __name__ == '__main__':
    main()
