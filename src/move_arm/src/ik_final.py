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


        NUM_SAMPLES = 10_000
        CUSTOM_BETA = 0.25 # idk what this should be yet. Recall 0 means no magnetic force
        MAX_DIST_POLE = 1 # meters
        POLE_POINT_1 = (0.689, -1.000, 0) # TODO: vision
        POLE_POINT_2 = (0.689, -1.000, 2) # TODO: vision
        GOAL = (0.689, -0.300, 0.382)     # TODO: vision
        NUM_STEPS = 10 # the number of increasing spheres

        
        STEP_SIZE = int(MAX_DIST_POLE / NUM_STEPS)

        motion_planner = MotionPlanner((-MAX_DIST_POLE, MAX_DIST_POLE), NUM_SAMPLES)
        motion_planner.print_points()

        optimized = motion_planner.optimize(GOAL, POLE_POINT_1, POLE_POINT_2, CUSTOM_BETA)
        print("Optimized:", optimized)


        # for each increasing sphere...
        for step in range(STEP_SIZE):
            motion_planner = MotionPlanner((-step * STEP_SIZE, step * STEP_SIZE), NUM_SAMPLES)
            motion_planner.print_points()

        # Set the desired orientation for the end effector HERE
        request.ik_request.pose_stamped.pose.position.x = 0.689
        request.ik_request.pose_stamped.pose.position.y = -0.300
        request.ik_request.pose_stamped.pose.position.z = 0.382 

        # request.ik_request.pose_stamped.pose.position.x = optimized[0]
        # request.ik_request.pose_stamped.pose.position.y = optimized[1]
        # request.ik_request.pose_stamped.pose.position.z = optimized[2] 

        request.ik_request.pose_stamped.pose.orientation.x = 0.0
        request.ik_request.pose_stamped.pose.orientation.y = 1.0
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w = 0.0
        

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






        return
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
