import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import pyaudio
from std_msgs.msg import Int16MultiArray

class MicPlotter(Node):
    def __init__(self):
        super().__init__('mic_plotter_node')

        # ROS Publisher (optional)
        self.publisher_ = self.create_publisher(Int16MultiArray, 'audio_pcm', 10)

        # Audio stream parameters
        self.chunk = 1024
        self.rate = 16000
        self.channels = 1
        self.format = pyaudio.paInt16

        # Initialize PyAudio stream
        self.audio = pyaudio.PyAudio()
        self.device_index = self.find_logitech_device()
        
        self.stream = self.audio.open(format=self.format,
                                      channels=self.channels,
                                      rate=self.rate,
                                      input=True,
                                      frames_per_buffer=self.chunk,
                                      input_device_index=self.device_index)

        # Initialize plot buffer
        self.buffer = np.zeros(4096, dtype=np.int16)
        self.latest_chunk = np.zeros(self.chunk, dtype=np.int16)

        # Setup real-time plot
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot(self.buffer)
        self.ax.set_ylim(-32768, 32767)
        self.ax.set_xlim(0, len(self.buffer))
        self.ax.set_title("Live Audio Signal from Webcam Mic")
        self.ax.set_xlabel("Sample Index")
        self.ax.set_ylabel("Amplitude")

        # Start the matplotlib animation and ROS timer
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=100)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('MicPlotter node initialized and streaming')

    def timer_callback(self):
        # Read and convert audio chunk
        data = self.stream.read(self.chunk, exception_on_overflow=False)
        self.latest_chunk = np.frombuffer(data, dtype=np.int16)

        # Publish to ROS topic
        msg = Int16MultiArray()
        msg.data = self.latest_chunk.tolist()
        self.publisher_.publish(msg)
        
    def find_logitech_device(self):
        for i in range(self.audio.get_device_count()):
            info = self.audio.get_device_info_by_index(i)
            if "USB Device" in info['name']:
                self.get_logger().info(f"Using audio device: {info['name']} (index {i})")
                return i
        raise RuntimeError("Logitech audio device not found!")

    def update_plot(self, _):
        # Shift old data, insert new chunk
        self.buffer = np.roll(self.buffer, -len(self.latest_chunk))
        self.buffer[-len(self.latest_chunk):] = self.latest_chunk
        self.line.set_ydata(self.buffer)
        return self.line,

    def destroy_node(self):
        self.stream.stop_stream()
        self.stream.close()
        self.p.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MicPlotter()
    try:
        plt.show()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
