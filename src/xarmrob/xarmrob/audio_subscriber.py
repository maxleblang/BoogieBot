# -*- coding: utf-8 -*-
"""
Created on Tue Apr 22 13:53:35 2025

@author: gageb
"""

# audio_subscriber.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray
import wave
import os
from datetime import datetime


class AudioSubscriber(Node):
    def __init__(self):
        super().__init__('audio_subscriber')

        self.subscription = self.create_subscription(
            ByteMultiArray,
            'audio_data',
            self.listener_callback,
            10)

        # WAV setup
        self.channels = 1
        self.rate = 16000
        self.sample_width = 2  # 16-bit audio (2 bytes)
        self.chunk = 1024
        self.filename = f"recording_{datetime.now().strftime('%Y%m%d_%H%M%S')}.wav"
        self.wav_file = wave.open(self.filename, 'wb')
        self.wav_file.setnchannels(self.channels)
        self.wav_file.setsampwidth(self.sample_width)
        self.wav_file.setframerate(self.rate)

        self.get_logger().info(f"Saving audio to {self.filename}")

    def listener_callback(self, msg: ByteMultiArray):
        audio_data = bytes(msg.data)
        self.wav_file.writeframes(audio_data)

    def destroy_node(self):
        self.wav_file.close()
        self.get_logger().info("WAV file closed")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AudioSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
