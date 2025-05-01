#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32MultiArray
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class BPMVisualizationNode(Node):
    """
    A node that subscribes to BPM and visualization data and displays it.
    This is particularly useful for development and debugging on a monitor
    connected to the Raspberry Pi.
    """
    
    def __init__(self):
        super().__init__('bpm_visualization_node')
        
        # Create a bridge between ROS and OpenCV
        self.bridge = CvBridge()
        
        # Subscribe to BPM data
        self.bpm_subscription = self.create_subscription(
            Int32,
            'bpm',
            self.bpm_callback,
            10
        )
        
        # Subscribe to waveform and spectrum images
        self.waveform_img_subscription = self.create_subscription(
            Image,
            'waveform_image',
            self.waveform_img_callback,
            10
        )
        
        self.spectrum_img_subscription = self.create_subscription(
            Image,
            'spectrum_image',
            self.spectrum_img_callback,
            10
        )
        
        # Store latest values
        self.current_bpm = 0
        self.waveform_img = None
        self.spectrum_img = None
        
        # Create display windows
        cv2.namedWindow('BPM Monitor', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('BPM Monitor', 1024, 600)
        
        # Timer for updating display
        self.display_timer = self.create_timer(0.1, self.update_display)
        
        self.get_logger().info('BPM Visualization Node started')
    
    def bpm_callback(self, msg):
        self.current_bpm = msg.data
    
    def waveform_img_callback(self, msg):
        try:
            self.waveform_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error: {str(e)}')
    
    def spectrum_img_callback(self, msg):
        try:
            self.spectrum_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error: {str(e)}')
    
    def update_display(self):
        # Create a canvas for display
        if self.waveform_img is not None and self.spectrum_img is not None:
            # Convert from RGB to BGR for OpenCV
            waveform_bgr = cv2.cvtColor(self.waveform_img, cv2.COLOR_RGB2BGR)
            spectrum_bgr = cv2.cvtColor(self.spectrum_img, cv2.COLOR_RGB2BGR)
            
            # Combine images vertically
            display_img = np.vstack((waveform_bgr, spectrum_bgr))
            
            # Add BPM text overlay
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(
                display_img, 
                f'Current BPM: {self.current_bpm}', 
                (20, 30), 
                font, 
                1, 
                (0, 255, 0), 
                2, 
                cv2.LINE_AA
            )
            
            # Display the image
            cv2.imshow('BPM Monitor', display_img)
            cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BPMVisualizationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
