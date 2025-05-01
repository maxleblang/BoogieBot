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
        self.bpm_subscriber = self.create_subscription(Int32, 'bpm', self.set_bpm, 1)
                

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
        self.simple_dance = self.interpolate_joint_angles(
            [0., -1.5707, 1.5707, 0., 0., 0., 0.],
            [0., -1.5707, 1.3707, 0., 0., 0., 0.],
            4
        )



        self.selected_dance = self.simple_dance

        # Set initial default rate to 126 BPM
        self.timer = self.create_timer(60/(126 * len(self.selected_dance)), self.boogie)


    # Callback to publish the pose at the specified rate. 
    def boogie(self):
        self.idx = (self.idx + 1) % len(self.selected_dance)
        self.joint_angles_desired_msg.position = self.selected_dance[self.idx]
        self.joint_angles_desired_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_joint_angles_desired.publish(self.joint_angles_desired_msg)


    # Callback the sets timer bpm to input
    def set_bpm(self, msg_in):
        bpm = int(msg_in.data)

        # Maybe check if bpm is within a tolerance before reconfiguring timer
        self.timer.cancel()
        self.timer = self.create_timer(60/(bpm*len(self.selected_dance)), self.boogie)


    # linearly interpolate between angles
    @staticmethod
    def interpolate_joint_poses(start_pose, end_pose, num_steps=100):
        # Convert inputs to numpy arrays
        start_pose = np.array(start_pose)
        end_pose = np.array(end_pose)
        
        # Create trajectory array
        trajectory = []
        
        for step in range(num_steps):
            # Calculate interpolation parameter (0 to 1)
            t = step / (num_steps - 1)
            
            # Linear interpolation
            interpolated = start_pose + t * (end_pose - start_pose)
            
            trajectory.append(interpolated)
        
        return np.array(trajectory)


def main(args=None):
    try: 
        rclpy.init(args=args)
        endpoint_automatic_instance = BoogieCommander()  
        rclpy.spin(endpoint_automatic_instance)
        
    except: 
        traceback.print_exc(limit=1)
        


if __name__ == '__main__':
    main()