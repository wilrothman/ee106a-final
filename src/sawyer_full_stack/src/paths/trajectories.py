#!/usr/bin/env/python

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D

"""
Set of classes for defining SE(3) trajectories for the end effector of a robot 
manipulator
"""

class Trajectory:
    def __init__(self, total_time):
        """
        Parameters
        ----------
        total_time : float
        	desired duration of the trajectory in seconds 
        """
        self.total_time = total_time

    def target_pose(self, time):
        """
        Returns where the arm end effector should be at time t, in the form of a 
        7D vector [x, y, z, qx, qy, qz, qw]. i.e. the first three entries are 
        the desired end-effector position, and the last four entries are the 
        desired end-effector orientation as a quaternion, all written in the 
        world frame.

        Hint: The end-effector pose with the gripper pointing down corresponds 
        to the quaternion [0, 1, 0, 0]. 

        Parameters
        ----------
        time : float        
    
        Returns
        -------
        7x' :obj:`numpy.ndarray`
            desired configuration in workspace coordinates of the end effector
        """
        pass

    def target_velocity(self, time):
        """
        Returns the end effector's desired body-frame velocity at time t as a 6D
        twist. Note that this needs to be a rigid-body velocity, i.e. a member 
        of se(3) expressed as a 6D vector.

        The function get_g_matrix from utils may be useful to perform some frame
        transformations.

        Parameters
        ----------
        time : float

        Returns
        -------
        6x' :obj:`numpy.ndarray`
            desired body-frame velocity of the end effector
        """
        pass

    def display_trajectory(self, num_waypoints=67, show_animation=False, save_animation=False):
        """
        Displays the evolution of the trajectory's position and body velocity.

        Parameters
        ----------
        num_waypoints : int
            number of waypoints in the trajectory
        show_animation : bool
            if True, displays the animated trajectory
        save_animatioon : bool
            if True, saves a gif of the animated trajectory
        """
        trajectory_name = self.__class__.__name__
        times = np.linspace(0, self.total_time, num=num_waypoints)
        target_positions = np.vstack([self.target_pose(t)[:3] for t in times])
        target_velocities = np.vstack([self.target_velocity(t)[:3] for t in times])
        
        fig = plt.figure(figsize=plt.figaspect(0.5))
        colormap = plt.cm.brg(np.fmod(np.linspace(0, 1, num=num_waypoints), 1))

        # Position plot
        ax0 = fig.add_subplot(1, 2, 1, projection='3d')
        pos_boundaries = [[-2, 2],
                           [-2, 2],
                           [-2, 2]]
        pos_padding = [[-0.1, 0.1],
                        [-0.1, 0.1],
                        [-0.1, 0.1]]
        ax0.set_xlim3d([max(pos_boundaries[0][0], min(target_positions[:, 0]) + pos_padding[0][0]), 
                        min(pos_boundaries[0][1], max(target_positions[:, 0]) + pos_padding[0][1])])
        ax0.set_xlabel('X')
        ax0.set_ylim3d([max(pos_boundaries[1][0], min(target_positions[:, 1]) + pos_padding[1][0]), 
                        min(pos_boundaries[1][1], max(target_positions[:, 1]) + pos_padding[1][1])])
        ax0.set_ylabel('Y')
        ax0.set_zlim3d([max(pos_boundaries[2][0], min(target_positions[:, 2]) + pos_padding[2][0]), 
                        min(pos_boundaries[2][1], max(target_positions[:, 2]) + pos_padding[2][1])])
        ax0.set_zlabel('Z')
        ax0.set_title("%s evolution of\nend-effector's position." % trajectory_name)
        line0 = ax0.scatter(target_positions[:, 0], 
                        target_positions[:, 1], 
                        target_positions[:, 2], 
                        c=colormap,
                        s=2)

        # Velocity plot
        ax1 = fig.add_subplot(1, 2, 2, projection='3d')
        vel_boundaries = [[-2, 2],
                           [-2, 2],
                           [-2, 2]]
        vel_padding = [[-0.1, 0.1],
                        [-0.1, 0.1],
                        [-0.1, 0.1]]
        ax1.set_xlim3d([max(vel_boundaries[0][0], min(target_velocities[:, 0]) + vel_padding[0][0]), 
                        min(vel_boundaries[0][1], max(target_velocities[:, 0]) + vel_padding[0][1])])
        ax1.set_xlabel('X')
        ax1.set_ylim3d([max(vel_boundaries[1][0], min(target_velocities[:, 1]) + vel_padding[1][0]), 
                        min(vel_boundaries[1][1], max(target_velocities[:, 1]) + vel_padding[1][1])])
        ax1.set_ylabel('Y')
        ax1.set_zlim3d([max(vel_boundaries[2][0], min(target_velocities[:, 2]) + vel_padding[2][0]), 
                        min(vel_boundaries[2][1], max(target_velocities[:, 2]) + vel_padding[2][1])])
        ax1.set_zlabel('Z')
        ax1.set_title("%s evolution of\nend-effector's translational body-frame velocity." % trajectory_name)
        line1 = ax1.scatter(target_velocities[:, 0], 
                        target_velocities[:, 1], 
                        target_velocities[:, 2], 
                        c=colormap,
                        s=2)

        if show_animation or save_animation:
            def func(num, line):
                line[0]._offsets3d = target_positions[:num].T
                line[0]._facecolors = colormap[:num]
                line[1]._offsets3d = target_velocities[:num].T
                line[1]._facecolors = colormap[:num]
                return line

            # Creating the Animation object
            line_ani = animation.FuncAnimation(fig, func, frames=num_waypoints, 
                                                          fargs=([line0, line1],), 
                                                          interval=max(1, int(1000 * self.total_time / (num_waypoints - 1))), 
                                                          blit=False)
        plt.show()
        if save_animation:
            line_ani.save('%s.gif' % trajectory_name, writer='pillow', fps=60)
            print("Saved animation to %s.gif" % trajectory_name)

class LinearTrajectory(Trajectory):
    def __init__(self, start_position, goal_position, total_time):

        Trajectory.__init__(self, total_time)
        self.start_position = start_position
        self.goal_position = goal_position
        self.distance = self.goal_position - self.start_position
        self.acceleration = (self.distance * 4.0) / (self.total_time ** 2) # keep constant magnitude acceleration
        self.v_max = (self.total_time / 2.0) * self.acceleration # maximum velocity magnitude
        self.desired_orientation = np.array([0, 1, 0, 0])

    def target_pose(self, time):
        """
        Returns where the arm end effector should be at time t, in the form of a 
        7D vector [x, y, z, qx, qy, qz, qw]. i.e. the first three entries are 
        the desired end-effector position, and the last four entries are the 
        desired end-effector orientation as a quaternion, all written in the 
        world frame.

        Hint: The end-effector pose with the gripper pointing down corresponds 
        to the quaternion [0, 1, 0, 0]. 

        Parameters
        ----------
        time : float        
    
        Returns
        -------
        7x' :obj:`numpy.ndarray`
            desired configuration in workspace coordinates of the end effector
        """
        t_to_max = self.v_max / self.acceleration
        x_at_max = 1/2 * self.acceleration[0] * t_to_max[0]*2
        y_at_max = 1/2 * self.acceleration[1] * t_to_max[1]*2
        z_at_max = 1/2 * self.acceleration[2] * t_to_max[2]*2


        if time <= self.total_time / 2.0:
            if time <= t_to_max[0]:
                x = 1/2 * self.acceleration[0] * time**2
            else:
                x = x_at_max + self.v_max[0] * (time - t_to_max[0])

            if time <= t_to_max[1]:
                y = 1/2 * self.acceleration[1] * time**2
            else:
                y = y_at_max + self.v_max[1] * (time - t_to_max[1])

            
            if time <= t_to_max[2]:
                z = 1/2 * self.acceleration[2] * time**2
            else:
                z = z_at_max + self.v_max[2] * (time - t_to_max[2])

            print(f"p_({time} < {self.total_time}/2) = {1/2 * self.acceleration * time ** 2}")
            # print(f"v_({time} < {self.total_time}/2) = {self.acceleration * time}")
            pos = self.start_position + np.array([x, y, z])

        else:
            p_at_half = self.start_position + 1/2 * self.acceleration * (1/2 * self.total_time) ** 2
            v_at_half = self.acceleration * (1/2 * self.total_time)
            # print(f"p_1/2 = {p_at_half}")
            # print(f"v_1/2 = {v_at_half}")

            pos = p_at_half + v_at_half * (time - 1/2 * self.total_time) + 1/2 * (-self.acceleration) * (time - 1/2 * self.total_time) ** 2
            print(f"p_({time} > {self.total_time}/2) = {pos}")

            # TODO: Calculate the position of the end effector at time t, 
            # For the second half of the trajectory, maintain a constant acceleration
            # Hint: Calculate the remaining distance to the goal position. 
            # print([self.v_max[0], self.acceleration[0] * t])
            # if self.total_time - time <= t_to_max[0]:
            #     x = 1/2 * self.acceleration[0] * time**2 - 1/2 * self.acceleration[0] * (time - t_to_max[0] / 2)**2
            # else:
            #     x = x_at_max + self.v_max[0] * (time - t_to_max[0])

            # if self.total_time - time <= t_to_max[1]:
            #     y = 1/2 * self.acceleration[1] * time**2 - 1/2 * self.acceleration[1] * (time - t_to_max[0] / 2)**2
            # else:
            #     y = y_at_max + self.v_max[1] * (time - t_to_max[1])

            
            # if time <= t_to_max[2]:
            #     self.total_time - time = 1/2 * self.acceleration[2] * time**2 - 1/2 * self.acceleration[2] * (time - t_to_max[0] / 2)**2
            # else:
                # z = z_at_max + self.v_max[2] * (time - t_to_max[2])

        print("DEBUG: returning:", np.hstack((pos, self.desired_orientation)))
        return np.hstack((pos, self.desired_orientation))

    def target_velocity(self, time):
        """
        Returns the end effector's desired body-frame velocity at time t as a 6D
        twist. Note that this needs to be a rigid-body velocity, i.e. a member 
        of se(3) expressed as a 6D vector.

        The function get_g_matrix from utils may be useful to perform some frame
        transformations.

        Parameters
        ----------
        time : float

        Returns
        -------
        6x' :obj:`numpy.ndarray`
            desired body-frame velocity of the end effector
        """
        if time <= self.total_time / 2.0:
            # TODO: calculate velocity using the acceleration and time
            # For the first half of the trajectory, we maintain a constant acceleration

            linear_vel = self.acceleration * time

        else:
            linear_vel = self.acceleration * (1/2 * self.total_time) + self.acceleration * (time - 1/2 * self.total_time)

            # TODO: start slowing the velocity down from the maximum one
            # For the second half of the trajectory, maintain a constant deceleration
            # time = self.total_time - time
            
            # if time <= t_to_max[0]:
            #     x = 1/2 * self.acceleration[0] * time**2
            # else:
            #     x = x_at_max + self.v_max[0] * (time - t_to_max[0])

            # if time <= t_to_max[1]:
            #     y = 1/2 * self.acceleration[1] * time**2
            # else:
            #     y = y_at_max + self.v_max[1] * (time - t_to_max[1])

            
            # if time <= t_to_max[2]:
            #     z = 1/2 * self.acceleration[2] * time**2
            # else:
            #     z = z_at_max + self.v_max[2] * (time - t_to_max[2])
                


            # if x <= x_at_max:
            #     x_dot = 1/2 * self.acceleration[0] * time ** 2
            # else:
            #     x_dot = self.v_max[0]

            # if y <= y_at_max:
            #     y_dot = 1/2 * self.acceleration[1] * time ** 2
            # else:
            #     y_dot = self.v_max[1]

            # if z <= z_at_max:
            #     z_dot = 1/2 * self.acceleration[2] * time ** 2
            # else:
            #     z_dot = self.v_max[2]

            # linear_vel = [0, 0, 0]
        print("DEBUG: returning vel:", np.hstack((linear_vel, np.zeros(3)))
)
        return np.hstack((linear_vel, np.zeros(3)))

class CircularTrajectory(Trajectory):
    def __init__(self, center_position, radius, total_time):
        Trajectory.__init__(self, total_time)
        self.center_position = center_position
        self.radius = radius
        self.angular_acceleration = (2 * np.pi * 4.0) / (self.total_time ** 2) # keep constant magnitude acceleration
        self.angular_v_max = (self.total_time / 2.0) * self.angular_acceleration # maximum velocity magnitude
        self.desired_orientation = np.array([0, 1, 0, 0])

    def target_pose(self, time):
        """
        Returns where the arm end effector should be at time t, in the form of a 
        7D vector [x, y, z, qx, qy, qz, qw]. i.e. the first three entries are 
        the desired end-effector position, and the last four entries are the 
        desired end-effector orientation as a quaternion, all written in the 
        world frame.

        Hint: The end-effector pose with the gripper pointing down corresponds 
        to the quaternion [0, 1, 0, 0]. 

        Parameters
        ----------
        time : float        
    
        Returns
        -------
        7x' :obj:`numpy.ndarray`
            desired configuration in workspace coordinates of the end effector
        """
        t_to_max = self.angular_v_max / self.angular_acceleration
        theta_at_max = 1/2 * self.angular_acceleration * t_to_max**2

        if time <= self.total_time / 2.0:
            # TODO: calculate the ANGLE of the end effector at time t, 
            # For the first half of the trajectory, maintain a constant acceleration

            if time <= t_to_max:
                theta = 1/2 * self.angular_acceleration * time**2
            else:
                theta = theta_at_max + self.angular_v_max * (time - t_to_max)

        else:
            # TODO: Calculate the ANGLE of the end effector at time t, 
            # For the second half of the trajectory, maintain a constant acceleration
            # Hint: Calculate the remaining angle to the goal position. 
            if self.total_time - time <= t_to_max:
                theta = 2*np.pi - (1/2 * self.angular_acceleration * (self.total_time - time)**2)
            else:
                theta = theta_at_max + self.angular_v_max * (time - t_to_max)
        
        pos_d = np.ndarray.flatten(self.center_position + self.radius * np.array([np.cos(theta), np.sin(theta), 0]))
        return np.hstack((pos_d, self.desired_orientation))


    def target_velocity(self, time):
        """
        Returns the end effector's desired body-frame velocity at time t as a 6D
        twist. Note that this needs to be a rigid-body velocity, i.e. a member 
        of se(3) expressed as a 6D vector.

        The function get_g_matrix from utils may be useful to perform some frame
        transformations.

        Parameters
        ----------
        time : float

        Returns
        -------
        6x' :obj:`numpy.ndarray`
            desired body-frame velocity of the end effector
        """
        t_to_max = self.angular_v_max / self.angular_acceleration
        theta_at_max = 1/2 * self.angular_acceleration * t_to_max**2

        if time <= self.total_time / 2.0:
            # TODO: calculate ANGULAR position and velocity using the acceleration and time
            # For the first half of the trajectory, we maintain a constant acceleration

            if time <= t_to_max:
                theta = 1/2 * self.angular_acceleration * time**2
            else:
                theta = theta_at_max + self.angular_v_max * (time - t_to_max)

            # print(f"theta: {theta}\ntheta_at_max: {theta_at_max}")
            if theta <= theta_at_max:
                theta_dot = (2 * self.angular_acceleration * theta) ** 1/2
            else:
                theta_dot = self.angular_v_max
        else:
            # TODO: start slowing the ANGULAR velocity down from the maximum one
            # For the second half of the trajectory, maintain a constant deceleration
            if self.total_time - time <= t_to_max:
                theta = 2*np.pi - (1/2 * self.angular_acceleration * (self.total_time - time)**2)
            else:
                theta = theta_at_max + self.angular_v_max * (time - t_to_max)

                        
            if 2*np.pi - theta <= theta_at_max:
                theta_dot = (2 * self.angular_acceleration * (2*np.pi - theta)) ** 1/2
            else:
                theta_dot = self.angular_v_max
            
        print(f"theta = {theta}\ntheta_dot = {theta_dot}\n")
        
        vel_d = np.ndarray.flatten(self.radius * theta_dot * np.array([-np.sin(theta), np.cos(theta), 0]))
        return np.hstack((vel_d, np.zeros(3)))

if __name__ == '__main__':
    """
    Run this file to visualize plots of your paths. Note: the provided function
    only visualizes the end effector position, not its orientation. Use the 
    animate function to visualize the full trajectory in a 3D plot.
    """

    # path = LinearTrajectory(np.array([0, 0, 0]), np.array([.1, .1, .1]), 10)
    path = CircularTrajectory(np.array([0.2, 0.4, 0.6]), .3, 10)
    path.display_trajectory()
