import rclpy
from rclpy.node import Node
import numpy as np
import pyaudio
from std_msgs.msg import Int16MultiArray

class MicPublisher(Node):
    def __init__(self):
        super().__init__('mic_publisher')

        self.publisher_ = self.create_publisher(Int16MultiArray, 'audio_pcm', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

        self.chunk = 1024
        self.rate = 16000
        self.channels = 1
        self.format = pyaudio.paInt16

        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(format=self.format,
                                  channels=self.channels,
                                  rate=self.rate,
                                  input=True,
                                  frames_per_buffer=self.chunk)

        self.get_logger().info('Mic publisher initialized')

    def timer_callback(self):
        data = self.stream.read(self.chunk, exception_on_overflow=False)
        signal = np.frombuffer(data, dtype=np.int16)

        msg = Int16MultiArray()
        msg.data = signal.tolist()

        self.publisher_.publish(msg)
        self.get_logger().info('Published audio chunk of size %d' % len(msg.data))

    def destroy_node(self):
        self.stream.stop_stream()
        self.stream.close()
        self.p.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    mic_node = MicPublisher()
    try:
        rclpy.spin(mic_node)
    except KeyboardInterrupt:
        pass
    mic_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
