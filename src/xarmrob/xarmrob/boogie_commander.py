#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Updated on 2025-04-23

@author: mleblang
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32


class BoogieCommander(Node):

    def __init__(self):
        super().__init__('boogie_commander')
        self.subscription = self.create_subscription(Int32, 'bpm', self.boogie_at_bpm, 1)

    def boogie_at_bpm(self, msg_in): 
        bpm = int(msg_in.data)
        self.get_logger().info(f"Received BPM of {bpm}")

        # TODO: Have static list of dance angles to loop through
        # TODO: Convert bpm to speed
        # TODO: Publish to proper topics to make the robot dance at the correct BMP
        

def main(args=None):
    rclpy.init(args=args)
    boogie_commander = BoogieCommander()

    try:
        # "spin" will block the program until an error occurs, e.g. keyboard break Ctrl+C. 
        rclpy.spin(boogie_commander)
    except: 
        # When an error occurs, catch it with "except" and stop the motors
        boogie_commander.get_logger().info('Stopping Boogie Commander') 

    rclpy.shutdown()


# Section to start the execution.  
if __name__ == "__main__":
    main()
