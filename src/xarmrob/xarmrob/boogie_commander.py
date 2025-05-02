#!/usr/bin/env python3

# ROS node to command an Endpoint Path to a HiWonder xArm 1S 
# Peter Adamczyk, University of Wisconsin - Madison
# Updated 2024-11-12

import rclpy
from rclpy.node import Node 
# from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup 
# from rclpy.executors import MultiThreadedExecutor
# import threading
import numpy as np
import traceback
import time
# from sensor_msgs.msg import JointState
# from xarmrob_interfaces.srv import ME439XArmInverseKinematics #, ME439XArmForwardKinematics
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState 


## Define a temporary function using Python "lambda" functionality to print colored text
# see https://stackoverflow.com/questions/287871/how-do-i-print-colored-text-to-the-terminal/3332860#3332860
# search that page for "CircuitSacul" to find that answer
coloredtext = lambda r, g, b, text: f'\033[38;2;{r};{g};{b}m{text}\033[38;2;255;255;255m'



class BoogieCommander(Node): 
    def __init__(self): 
        super().__init__('boogie_commander')

        # publish the joint angles here
        self.pub_joint_angles_desired = self.create_publisher(JointState,'/joint_angles_desired',1)
        # Get the bpm here
        self.bpm_subscriber = self.create_subscription(Int32, 'beat', self.set_bpm, 1)
        self.idx = 0
                

        # Starting pose
        self.joint_neutral_angs_base_to_tip = self.declare_parameter('joint_neutral_angs_base_to_tip', [0., -1.5707, 1.5707, 0., 0., 0., 0.]).value
        self.angle = list(map(int,self.joint_neutral_angs_base_to_tip))  # List of neutral values for each bus servo
        # Create desired angle message
        self.joint_angles_desired_msg = JointState()
        self.joint_angles_desired_msg.name = ['base_joint', 'shoulder_joint', 'elbow_joint', 'forearm_joint', 'wrist_joint', 'fingers_joint', 'gripper']
        self.joint_angles_desired_msg.position = self.angle
        # Initialize to starting angle
        self.joint_angles_desired_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_joint_angles_desired.publish(self.joint_angles_desired_msg)

        # Dance initialization stuff
        self.simple_dance = np.vstack((
            [np.pi/4,-np.pi/2, np.pi/2,-np.pi/4, -np.pi/4,0.,0.],
            [np.pi/8,-np.pi/2, np.pi/2,-np.pi/8,-np.pi/8,0.,0.],
            [0.,-np.pi/2, np.pi/2,0.,0,0.,0.],
            [-np.pi/8,-np.pi/2, np.pi/2,np.pi/8,-np.pi/8,0.,0.],
            [-np.pi/4,-np.pi/2, np.pi/2,np.pi/4,-np.pi/4,0.,0.],
            [-np.pi/8,-np.pi/2, np.pi/2,np.pi/8,-np.pi/8,0.,0.],
            [0.,-np.pi/2, np.pi/2,0.,0.,0.,0.],
            [np.pi/8,-np.pi/2, np.pi/2,-np.pi/8,-np.pi/8,0.,0.],
        ))
        
        self.simple_dance = self.interpolate_minimum_jerk_poses_only(self.simple_dance)


        self.selected_dance = self.simple_dance
        self.current_bpm = 117
        self.prev_bpm = 117
        self.bpm_delta = 31

        # Set initial default rate to 126 BPM
        self.timer = None


    # Callback to publish the pose at the specified rate. 
    def boogie(self):
        self.idx = (self.idx + 1) % len(self.selected_dance)
        self.joint_angles_desired_msg.position = self.selected_dance[self.idx]
        self.joint_angles_desired_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_joint_angles_desired.publish(self.joint_angles_desired_msg)


    # Callback the sets timer bpm to input
    def set_bpm(self, msg_in):
        self.current_bpm = int(msg_in.data)
    
        if 117 <= self.current_bpm <= 123:
            if abs(self.current_bpm - self.prev_bpm) >= 2:
                # Maybe check if bpm is within a tolerance before reconfiguring timer
                self.get_logger().info(f"Changing to new bpm of {self.current_bpm}")
                if self.timer:
                    self.timer.cancel()
                self.timer = self.create_timer(60/((self.current_bpm + self.bpm_delta)*len(self.selected_dance)), self.boogie)
                self.prev_bpm = self.current_bpm


    # linearly interpolate between angles
    @staticmethod
    def interpolate_joint_poses(poses, steps_between_poses=50, method="minimum_jerk"):
        """
        Interpolate between a sequence of joint poses using minimum jerk trajectory.
        
        Args:
            poses: List of joint angle arrays, where each array is [j1, j2, ..., jn]
            steps_between_poses: Number of steps between consecutive poses
            method: Interpolation method ("linear" or "minimum_jerk")
            
        Returns:
            Array of interpolated joint angles for the entire trajectory
        """
        if len(poses) < 2:
            raise ValueError("Need at least two poses to interpolate")
        
        # Convert all poses to numpy arrays
        poses = [np.array(pose) for pose in poses]
        
        # Create trajectory array
        trajectory = []
        
        # Interpolate between each consecutive pair of poses
        for i in range(len(poses) - 1):
            start_pose = poses[i]
            end_pose = poses[i + 1]
            
            # For each step between the current pose pair
            for step in range(steps_between_poses):
                # Don't add the end point except for the final segment
                if step == steps_between_poses - 1 and i < len(poses) - 2:
                    continue
                    
                # Calculate interpolation parameter (0 to 1)
                t = step / (steps_between_poses - 1)
                
                if method == "minimum_jerk":
                    # Minimum jerk trajectory (5th order polynomial)
                    # This polynomial ensures zero velocity and acceleration at endpoints
                    smooth_t = 10*t**3 - 15*t**4 + 6*t**5
                else:
                    # Default to linear interpolation
                    smooth_t = t
                
                # Interpolate between poses
                interpolated = start_pose + smooth_t * (end_pose - start_pose)
                
                trajectory.append(interpolated)
        
        return np.array(trajectory)
    

    @staticmethod
    def interpolate_minimum_jerk_poses_only(pose_list, endpoint_speed=.3, command_frequency=120):
        """
        Interpolates between a list of poses using minimum jerk and returns only the poses.

        Args:
            pose_list (np.ndarray): (N, D) array of waypoints.
            endpoint_speed (float): Desired speed (units/sec).
            command_frequency (float): Command rate (Hz).

        Returns:
            trajectory (np.ndarray): (M, D) array of interpolated poses.
        """
        pose_list = np.array(pose_list)
        all_poses = []

        for i in range(len(pose_list) - 1):
            pos_init = pose_list[i]
            pos_end = pose_list[i + 1]
            
            displacement = pos_end - pos_init
            distance = np.linalg.norm(displacement)
            duration_nom = distance / endpoint_speed 
            nsteps = int(np.ceil(duration_nom * command_frequency))
            t_rel = np.arange(nsteps) / nsteps

            min_jerk_traj = 10 * t_rel**3 - 15 * t_rel**4 + 6 * t_rel**5
            disp_traj = np.column_stack([
                p_i + disp * min_jerk_traj
                for p_i, disp in zip(pos_init, displacement)
            ])

            all_poses.append(disp_traj)

        trajectory = np.vstack(all_poses)
        return trajectory

def main(args=None):
    try: 
        rclpy.init(args=args)
        boogie_commander = BoogieCommander()  
        rclpy.spin(boogie_commander)
        
    except: 
        traceback.print_exc(limit=1)
        


if __name__ == '__main__':
    main()