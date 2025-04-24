# -*- coding: utf-8 -*-
"""
Created on Tue Apr 22 13:46:35 2025

@author: mleblang
"""

# audio_publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, ByteMultiArray


class BPMPublisher(Node):
    """
    This Node takes audio input from a Logitech webcam mic and extracts the BPM of the song being played.
    It then publishes the song's bpm to the bpm topic for BoogieCommander to use
    """
    def __init__(self):
        super().__init__('bpm_publisher')
        self.publisher = self.create_publisher(Int32, 'bpm', 1)
        self.subscription = self.create_subscription(ByteMultiArray, 'raw_audio', self.extract_bpm_from_audio, 10)

    def extract_bpm_from_audio(self, msg_in):
        raw_audio = bytes(msg_in.date)
        # TODO: process raw_audio to get bmp
        # TODO: publish detected bpm (maybe don't publish if the bpm is within certain tolerance of previous bpm)



        bpm = 100
        msg_out = Int32()
        msg_out.data = bpm
        self.publisher.publish(msg_out)
        self.get_logger().info(f"Detected BPM of {bpm}")

def main(args=None):
    rclpy.init(args=args)
    bpm_publisher = BPMPublisher()

    try:
        # "spin" will block the program until an error occurs, e.g. keyboard break Ctrl+C. 
        rclpy.spin(bpm_publisher)
    except: 
        # When an error occurs, catch it with "except" and stop the motors
        bpm_publisher.get_logger().info('Stopping BPM Publisher') 

    rclpy.shutdown()


if __name__ == '__main__':
    main()
