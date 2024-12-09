#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
import sys
from model import *

def main():
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


        START_EF_POSITION = (0.816, 0.161, 0.642) # TODO: fix with a subscriber
        NUM_SAMPLES = 10_000
        CUSTOM_BETA = 0.25 # idk what this should be yet. Recall 0 means no magnetic force
        MAX_DIST_POLE = 1 # meters
        POLE_POINT_1 = (0.689, -1.000, 0) # TODO: vision
        POLE_POINT_2 = (0.689, -1.000, 2) # TODO: vision
        # Starting posn at regular tuck: (0.689,  0.161, 0.381)
        # Starting posn at custom  tuck: (0.816, -0.161, 0.642)
        GOAL = (0.689, -0.300, 0.382)     # TODO: vision
        NUM_STEPS = 5 # the number of increasing spheres

        
        STEP_SIZE = MAX_DIST_POLE / NUM_STEPS

        ef_posn = np.array(START_EF_POSITION)

        # for each increasing sphere...
        for step in range(1): # range(STEP_SIZE):
            # 1-index step or else you get 0 on the first step
            motion_planner = MotionPlanner(ef_posn, (step + 1) * STEP_SIZE, NUM_SAMPLES)
            motion_planner.print_points()
            print(STEP_SIZE)
            optimized = motion_planner.optimize(GOAL, POLE_POINT_1, POLE_POINT_2, CUSTOM_BETA)
            print(f"Optimized (step {step}):", optimized)
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
            group.set_pose_target(req/robot/joint_statesuest.ik_request.pose_stamped)

            # TRY THIS
            # Setting just the position without specifying the orientation
            ###group.set_position_target([0.5, 0.5, 0.0])

            # Plan IK
            plan = group.plan()


        try:
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
            user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
            
            # Execute IK if safe
            if user_input == 'y':
                group.execute(plan[1])
            
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

# Python's syntax for a main() method
if __name__ == '__main__':
    main()
