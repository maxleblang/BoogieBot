# -*- coding: utf-8 -*-
"""
Created on Tue Apr 22 13:46:35 2025

@author: gageb
"""

# audio_publisher.py

import rclpy
from rclpy.node import Node
import pyaudio
import numpy as np
from std_msgs.msg import ByteMultiArray


class AudioPublisher(Node):
    def __init__(self):
        super().__init__('audio_publisher')

        # PyAudio setup
        self.chunk = 1024  # number of audio frames per buffer
        self.format = pyaudio.paInt16
        self.channels = 1
        self.rate = 16000  # 16kHz

        self.audio = pyaudio.PyAudio()

        # Find Logitech device (adjust name matching as needed)
        self.device_index = self.find_logitech_device()

        self.stream = self.audio.open(format=self.format,
                                      channels=self.channels,
                                      rate=self.rate,
                                      input=True,
                                      frames_per_buffer=self.chunk,
                                      input_device_index=self.device_index)

        self.publisher_ = self.create_publisher(ByteMultiArray, 'audio_data', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz

        self.get_logger().info("Audio Publisher Node Started")

    def find_logitech_device(self):
        for i in range(self.audio.get_device_count()):
            info = self.audio.get_device_info_by_index(i)
            if "Logitech" in info['name']:
                self.get_logger().info(f"Using audio device: {info['name']} (index {i})")
                return i
        raise RuntimeError("Logitech audio device not found!")

    def timer_callback(self):
        data = self.stream.read(self.chunk, exception_on_overflow=False)
        msg = ByteMultiArray()
        msg.data = list(data)  # send raw bytes
        self.publisher_.publish(msg)

    def destroy_node(self):
        self.stream.stop_stream()
        self.stream.close()
        self.audio.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AudioPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
