#!/usr/bin/env python3

# ROS node to command an Endpoint Path to a HiWonder xArm 1S 
# Peter Adamczyk, University of Wisconsin - Madison
# Updated 2024-11-12
# Modified to move to each dance pose on beat

import rclpy
from rclpy.node import Node 
import numpy as np
import traceback
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
        self.pub_joint_angles_desired = self.create_publisher(JointState, '/joint_angles_desired', 10)
        
        # Subscribe to the beat topic - this will trigger dance movements
        self.beat_subscriber = self.create_subscription(Int32, 'beat', self.beat_callback, 10)
        
        # Initialize dance sequence index
        self.idx = 0
                
        # Starting pose
        self.joint_neutral_angs_base_to_tip = self.declare_parameter('joint_neutral_angs_base_to_tip', [0., -1.5707, 1.5707, 0., 0., 0., 0.]).value
        self.angle = list(map(float, self.joint_neutral_angs_base_to_tip))  # List of neutral values for each bus servo
        
        # Create desired angle message
        self.joint_angles_desired_msg = JointState()
        self.joint_angles_desired_msg.name = ['base_joint', 'shoulder_joint', 'elbow_joint', 'forearm_joint', 'wrist_joint', 'fingers_joint', 'gripper']
        self.joint_angles_desired_msg.position = self.angle
        
        # Initialize to starting angle
        self.joint_angles_desired_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_joint_angles_desired.publish(self.joint_angles_desired_msg)

        # Dance initialization
        self.simple_dance = np.vstack((
            [np.pi/4, -np.pi/2, np.pi/2, -np.pi/8, -np.pi/8, 0., 0.],
            [0., -np.pi/2, np.pi/2, 0., 0, 0., 0.],
            [np.pi/4, -np.pi/2, np.pi/2, -np.pi/8, -np.pi/8, 0., 0.],
            [0., -np.pi/2, np.pi/2, 0., 0, 0., 0.],
        ))
        
        # Use the original dance poses without interpolation
        # We want to hit each pose exactly on the beat
        self.selected_dance = self.simple_dance
        
        # Log initialization
        self.get_logger().info(coloredtext(0, 255, 0, 'Boogie Commander initialized'))
        self.get_logger().info(f'Ready to dance on beat with {len(self.selected_dance)} poses')

    # Callback that executes when a beat message is received
    def beat_callback(self, msg_in):
        # Extract BPM value if needed (for logging/debugging)
        bpm = int(msg_in.data)
        
        # Move to the next pose in the dance sequence
        self.move_to_next_pose()
        
        # Log the beat and pose info
        self.get_logger().info(f'Beat received! BPM: {bpm}, Moving to pose {self.idx}')

    # Method to move to the next pose in the dance sequence
    def move_to_next_pose(self):
        # Update position in the dance sequence
        current_pose = self.selected_dance[self.idx]
        
        # Update the joint angles message
        self.joint_angles_desired_msg.position = current_pose
        self.joint_angles_desired_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Publish the new desired position
        self.pub_joint_angles_desired.publish(self.joint_angles_desired_msg)
        
        # Move to the next index in the dance sequence, looping back to 0 when we reach the end
        self.idx = (self.idx + 1) % len(self.selected_dance)

def main(args=None):
    try: 
        rclpy.init(args=args)
        boogie_commander = BoogieCommander()  
        rclpy.spin(boogie_commander)
        
    except: 
        traceback.print_exc(limit=1)

if __name__ == '__main__':
    main()