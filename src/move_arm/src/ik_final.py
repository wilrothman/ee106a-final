#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped, Point, Pose, Quaternion
from moveit_commander import MoveGroupCommander
import numpy as np
from tf.transformations import quaternion_from_euler
from numpy import linalg
import sys
from model import *
import roslaunch


class IKFinal:
    def __init__(self):

        self.POLE_POS = None
        self.LIGHT_POS = None

        # pole_sub  =  rospy.Subscriber("goal_point", Point, self.pole_callback)
        # light_sub = rospy.Subscriber("goal_point", Point, self.light_callback)

        # Wait for the IK service to become available
        rospy.wait_for_service('compute_ik')
        print("DEBUG started")
        rospy.init_node('service_query')
        # Create the function used to call the service
        compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
        while not rospy.is_shutdown():    
            #### SETUP ####    
            # Construct the request
            request = GetPositionIKRequest()
            request.ik_request.group_name = "right_arm"
            # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
            link = "right_gripper_tip"
            request.ik_request.ik_link_name = link
            # request.ik_request.attempts = 20
            request.ik_request.pose_stamped.header.frame_id = "base"
            #### /SETUP ####

            # print("Pole positions")
            # print(self.POLE_POS)

            START_EF_POSITION = (0.816, 0.161, 0.642) # TODO: fix with a subscriber
            NUM_SAMPLES = 10_000
            CUSTOM_BETA = 0.0125 # [0, 0.05]
            # POLE_POINT_1 = self.POLE_POS # TODO: vision
            # POLE_POINT_2 = (self.POLE_POS[0], self.POLE_POS[1], self.POLE_POS[2] + 0.85) # TODO: vision

            POLE_POINT_1 = (0.900, 0.161, 0) # TODO: vision
            POLE_POINT_2 = (0.900, 0.161, 2) # TODO: vision

            # Starting posn at regular tuck: (0.689,  0.161, 0.381)
            # Starting posn at custom  tuck: (0.816, -0.161, 0.642)
            # GOAL = self.LIGHT_POS     # TODO: vision

            GOAL = (0.950, 0.161, 0.100) 
            MAX_DIST_POLE = np.linalg.norm(np.array(START_EF_POSITION) - np.array(GOAL)) # meters

            NUM_STEPS = 10 # the number of increasing spheres
            
            STEP_SIZE = MAX_DIST_POLE / NUM_STEPS

            ef_posn = np.array(START_EF_POSITION)
            print(f"Pole is distance {MAX_DIST_POLE} from tuck.\nStep Size= {MAX_DIST_POLE}meters / {NUM_STEPS}steps = {STEP_SIZE}meters/step")

            waypoints = []
            # for each increasing sphere...
            for step in range(NUM_STEPS):
                # 1-index step or else you get 0 on the first step
                motion_planner = MotionPlanner(ef_posn, (step + 1) * STEP_SIZE, NUM_SAMPLES)
                # motion_planner.print_points()
                # print(STEP_SIZE)
                optimized = motion_planner.optimize(GOAL, POLE_POINT_1, POLE_POINT_2, CUSTOM_BETA)
                # Do not update optimized if the current point is more optimal
                def optimal_argmin(arr1, arr2, goal):
                    if type(arr1) is not np.array:
                        arr1 = np.array(arr1)
                    if type(arr2) is not np.array:
                        arr2 = np.array(arr2)

                    if np.linalg.norm(arr1 - goal) < np.linalg.norm(arr2 - goal):
                        return arr1
                    else:
                        return arr2
                
                ef_posn = optimal_argmin(ef_posn, optimized, GOAL)

                print(f"\nEF Position(step {step}):", optimized)
                print(f"Distance to goal (step {step}): {np.linalg.norm(ef_posn - np.array(GOAL))}")

                if all(ef_posn == optimized) or step == 0:
                    waypoints.append(ef_posn)
                    print("New waypoint added.")
                else:
                    print("Discarded waypoint candidate.")
                print()


                request.ik_request.pose_stamped.pose.position.x = optimized[0]
                request.ik_request.pose_stamped.pose.position.y = optimized[1]
                request.ik_request.pose_stamped.pose.position.z = optimized[2] 

                request.ik_request.pose_stamped.pose.orientation.x = -0.018
                request.ik_request.pose_stamped.pose.orientation.y = 0.742
                request.ik_request.pose_stamped.pose.orientation.z = -0.018
                request.ik_request.pose_stamped.pose.orientation.w = 0.670
            

                # Send the request to the service
                response = compute_ik(request)
                
                # Print the response HERE
                # print(response)
                group = MoveGroupCommander("right_arm")

                # Setting position and orientation target
                group.set_pose_target(request.ik_request.pose_stamped)

                # TRY THIS
                # Setting just the position without specifying the orientation
                ###group.set_position_target([0.5, 0.5, 0.0])

                # Plan IK
                plan = group.plan()


            try:
                group = MoveGroupCommander("right_arm")

                # Use the waypoints to generate a continuous Cartesian path
                (cartesian_plan, fraction) = group.compute_cartesian_path(
                    [Pose(
                        # header=Header(frame_id="base"),
                            position=Point(x=point[0], y=point[1], z=point[2]),
                            orientation=Quaternion(x=-0.018, y=0.742, z=-0.018, w=0.670)
                    ) for point in waypoints],  # List of waypoints
                    0.01,  # e.g., 1 cm resolution
                    0.0    # Jump threshold
                )

                if fraction < 1.0:
                    rospy.logwarn("Only a partial trajectory was computed. Fraction: {}".format(fraction))
                else:
                    rospy.loginfo("Successfully computed a full trajectory through all waypoints.")

                if fraction < 1.0:
                    rospy.logwarn("Only a partial trajectory was computed. Fraction: {}".format(fraction))
                else:
                    rospy.loginfo("Successfully computed a full trajectory through all waypoints.")

                # Execute the Cartesian path
                user_input = input("Enter 'y' if the trajectory looks safe on RVIZ: ")
                if user_input == 'y':
                    group.execute(cartesian_plan, wait=True)

                ### Path reversal ###

                reversed_waypoints = list(reversed(waypoints))

                # Plan the Cartesian path for the reversed waypoints
                (reverse_cartesian_plan, reverse_fraction) = group.compute_cartesian_path(
                    [Pose(
                        position=Point(x=point[0], y=point[1], z=point[2]),
                        orientation=Quaternion(x=-0.018, y=0.742, z=-0.018, w=0.670)
                    ) for point in reversed_waypoints],  # List of reversed waypoints
                    0.01,  # e.g., 1 cm resolution
                    0.0    # Jump threshold
                )

                if reverse_fraction < 1.0:
                    rospy.logwarn("Only a partial trajectory was computed for the reversed path. Fraction: {}".format(reverse_fraction))
                else:
                    rospy.loginfo("Successfully computed a full reversed trajectory to tuck position.")

                # Execute the reversed Cartesian path
                user_input = input("Enter 'y' if the reversed trajectory looks safe on RVIZ: ")
                if user_input == 'y':
                    group.execute(reverse_cartesian_plan, wait=True)

                rospy.loginfo("Starting the tuck process...")

                # Launch the tuck action
                uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                roslaunch.configure_logging(uuid)
                tuck_launch = roslaunch.parent.ROSLaunchParent(
                    uuid, ["src/move_arm/launch/custom_sawyer_tuck.launch"]
                )

                tuck_launch.start()
                rospy.loginfo("Tuck launch initiated.")

                # Wait for the tuck to complete or stop manually
                tuck_launch.spin()

                rospy.loginfo("Tuck process completed.")


                break

                # # Send the request to the service
                # response = compute_ik(request)
                
                # # Print the response HERE
                # # print(response)
                # group = MoveGroupCommander("right_arm")

                # # Setting position and orientation target
                # group.set_pose_target(request.ik_request.pose_stamped)

                # # TRY THIS
                # # Setting just the position without specifying the orientation
                ###group.set_position_target([0.5, 0.5, 0.0])

                # Plan IK
                # plan = group.plan()
                # user_input = input("Enter 'y' if the trajectory looks safe on RVIZ ")
                
                # # Execute IK if safe
                # if user_input == 'y':
                #     group.execute(plan[1])
                
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)

    # Coordinate deconstruction callback for pole subscriber
    def pole_callback(self, msg):
        self.POLE_POS = (msg.x + 0.2, msg.y, msg.z)
    
    # Coordinate deconstruction callback for light subscriber
    def light_callback(self, msg):
        self.LIGHT_POS = (msg.x + 0.1, msg.y, msg.z)


# Python's syntax for a main() method
if __name__ == '__main__':
    IKFinal()
