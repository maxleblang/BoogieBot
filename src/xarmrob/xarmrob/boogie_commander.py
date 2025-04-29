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
from xarmrob_interfaces.msg import ME439PointXYZ

import xarmrob.smooth_interpolation as smoo 


## Define a temporary function using Python "lambda" functionality to print colored text
# see https://stackoverflow.com/questions/287871/how-do-i-print-colored-text-to-the-terminal/3332860#3332860
# search that page for "CircuitSacul" to find that answer
coloredtext = lambda r, g, b, text: f'\033[38;2;{r};{g};{b}m{text}\033[38;2;255;255;255m'



class BoogieCommander(Node): 
    def __init__(self): 
        super().__init__('endpoint_automatic')
                
        
        self.xyz_goal = [0.15, 0.0, 0.10] # roughly upright neutral with wrist at 45 degrees. Formally: [0.1646718829870224, 0.0, 0.1546700894832611]
        self.old_xyz_goal = [0.15, 0.0, 0.10]
        self.xyz_traj = [self.old_xyz_goal]
        self.disp_traj = self.xyz_traj 
        self.gripper = 0
        self.idx = 0

        # =============================================================================
        #   # Publisher for the Endpoint goal. 
        # =============================================================================
        self.pub_endpoint_desired = self.create_publisher(ME439PointXYZ,'/endpoint_desired',1)#,callback_group=ReentrantCallbackGroup())
        # Create the message, with a nominal pose
        self.endpoint_desired_msg = ME439PointXYZ()
        self.endpoint_desired_msg.xyz = self.xyz_goal 

        # command frequency parameter: how often we expect it to be updated    
        self.command_frequency = self.declare_parameter('command_frequency',5).value
        self.movement_time_ms = round(1000/self.command_frequency)  # milliseconds for movement time. 
        self.endpoint_speed = self.declare_parameter('endpoint_speed',0.05).value  # nominal speed for continuous movement among points. 
        
        #Get Height offset for safety (do practice runs in the air)
        self.vertical_offset = self.declare_parameter('vertical_offset',0.02).value
        
        self.set_endpoint_trajectory()
        
        time.sleep(5)
        
        # Set up a timer to send the commands at the specified rate. 
        self.timer = self.create_timer(self.movement_time_ms/1000, self.send_endpoint_desired)#,callback_group=ReentrantCallbackGroup())

    # Callback to publish the endpoint at the specified rate. 
    def send_endpoint_desired(self):
        simple_dance = [
            [0.1,0.1,0.1],
            [0.1,-0.1,0.1]
        ]

        self.idx = (self.idx + 1) % len(simple_dance)
        self.endpoint_desired_msg.xyz = simple_dance[self.idx]
        self.pub_endpoint_desired.publish(self.endpoint_desired_msg)

        
        
    # def set_endpoint_trajectory(self):

    #     # Use an SVG file to specify the path: 
    #     import xarmrob.parse_svg_for_robot_arm_v03 as psvg
    #     endpoints = psvg.convert_svg_to_endpoints(self.filename, xlength=0.10, ylength=0.10, rmin=0.18, rmax=0.28)
    #     endpoints = np.vstack( (np.array(self.old_xyz_goal), endpoints, np.array(self.old_xyz_goal)))
    #     endpoints[:,2] = endpoints[:,2]+self.vertical_offset
    #     # print(endpoints)
        
    #     # Build a long array of many endpoint locations using smooth constant-speed control: 
    #     self.disp_traj = np.ndarray( (0,3) )
    #     for ii in range(1,len(endpoints)):
    #         # Do linear or minimum jerk interpolation
    #         t,seg_traj = smoo.constant_velocity_interpolation(np.array(endpoints[ii-1]), np.array(endpoints[ii]), self.endpoint_speed, self.command_frequency)
    #         self.disp_traj = np.vstack( (self.disp_traj,seg_traj) )
    #     self.idx = 0
    #     print(self.disp_traj)


def main(args=None):
    try: 
        rclpy.init(args=args)
        endpoint_automatic_instance = BoogieCommander()  
        rclpy.spin(endpoint_automatic_instance)
        
    except: 
        traceback.print_exc(limit=1)
        


if __name__ == '__main__':
    main()