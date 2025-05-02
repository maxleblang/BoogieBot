#!/usr/bin/env python3

import numpy as np
import pyaudio
import matplotlib as mpl
mpl.use('Agg')  # Non-interactive backend suitable for headless Raspberry Pi
import matplotlib.pyplot as plt
from scipy.signal import butter, lfilter, find_peaks
import threading
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class BPMDetector:
    def __init__(self, sample_rate=44100, chunk_size=1024, history_sec=6):
        self.sample_rate = sample_rate
        self.chunk_size = chunk_size
        self.buffer_len = int(sample_rate / chunk_size * history_sec)
        self.envelope_buffer = []
        self.last_bpm = 0
        self.recent_bpms = []

    def update(self, audio_chunk):
        rectified = np.abs(audio_chunk)
        smoothed = lfilter([1], [1, -0.997], rectified)
        mean_env = np.mean(smoothed)
        self.envelope_buffer.append(mean_env)
        if len(self.envelope_buffer) > self.buffer_len:
            self.envelope_buffer.pop(0)
        if len(self.envelope_buffer) >= 16:
            bpm = self.calculate_bpm()
            if 40 <= bpm <= 200:
                self.last_bpm = bpm
        return self.last_bpm

    def calculate_bpm(self):
        env = np.array(self.envelope_buffer)
        env = (env - np.mean(env)) / (np.std(env) + 1e-6)

        min_interval = int(0.35 * self.sample_rate / self.chunk_size)
        peaks, _ = find_peaks(env, height=1.2, distance=min_interval)

        if len(peaks) < 2:
            return self.last_bpm

        intervals = np.diff(peaks) * (self.chunk_size / self.sample_rate)
        if len(intervals) == 0 or np.median(intervals) == 0:
            return self.last_bpm

        bpm = 60.0 / np.median(intervals)

        self.recent_bpms.append(bpm)
        if len(self.recent_bpms) > 5:
            self.recent_bpms.pop(0)

        return int(np.median(self.recent_bpms))


class BPMDetectorNode(Node):
    def __init__(self):
        super().__init__('bpm_detector_node')
        
        # Parameters
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('chunk_size', 1024)
        self.declare_parameter('buffer_seconds', 5)
        self.declare_parameter('low_freq', 60)
        self.declare_parameter('high_freq', 250)
        self.declare_parameter('device_index', None)
        self.declare_parameter('visualization_rate', 5.0)  # Hz
        
        self.sample_rate = self.get_parameter('sample_rate').value
        self.chunk_size = self.get_parameter('chunk_size').value
        self.buffer_seconds = self.get_parameter('buffer_seconds').value
        self.low_freq = self.get_parameter('low_freq').value
        self.high_freq = self.get_parameter('high_freq').value
        
        # Initialize audio buffer
        self.buffer_size = int(self.buffer_seconds * self.sample_rate)
        self.audio_buffer = np.zeros(self.buffer_size)
        self.time_buffer = np.linspace(-self.buffer_seconds, 0, self.buffer_size)
        
        # FFT buffer
        self.fft_size = 2048
        self.fft_buffer = np.zeros(self.fft_size)
        self.freq_buffer = np.fft.rfftfreq(self.fft_size, d=1/self.sample_rate)
        
        # Create BPM detector
        self.bpm_detector = BPMDetector(
            sample_rate=self.sample_rate,
            chunk_size=self.chunk_size,
            history_sec=6
        )
        
        # Publishers
        self.bpm_publisher = self.create_publisher(Int32, 'bpm', 10)
        # self.waveform_publisher = self.create_publisher(Float32MultiArray, 'audio_waveform', 10)
        # self.spectrum_publisher = self.create_publisher(Float32MultiArray, 'audio_spectrum', 10)
        # self.waveform_img_publisher = self.create_publisher(Image, 'waveform_image', 10)
        # self.spectrum_img_publisher = self.create_publisher(Image, 'spectrum_image', 10)
        
        # Initialize PyAudio
        self.p = pyaudio.PyAudio()
        self.stream = None
        self.running = False
        
        # Bridge for converting images
        # self.bridge = CvBridge()
        
        # Create timers
        # timer_period = 1.0 / self.get_parameter('visualization_rate').value
        # self.timer = self.create_timer(timer_period, self.timer_callback)

        self.device_index = self.find_mic()
        
        # Start audio processing in a separate thread
        self.audio_thread = threading.Thread(target=self.start_audio)
        self.audio_thread.daemon = True
        self.audio_thread.start()
        
        self.get_logger().info('BPM Detector Node started')

    def _butter_bandpass(self, lowcut, highcut, fs, order=5):
        nyq = 0.5 * fs
        low = max(lowcut / nyq, 1e-5)
        high = min(highcut / nyq, 0.99999)
        b, a = butter(order, [low, high], btype='band')
        return b, a

    def _apply_bandpass_filter(self, data, lowcut, highcut, fs, order=3):
        b, a = self._butter_bandpass(lowcut, highcut, fs, order=order)
        return lfilter(b, a, data)

    def _audio_callback(self, in_data, frame_count, time_info, status):
        audio_data = np.frombuffer(in_data, dtype=np.float32)
        filtered = self._apply_bandpass_filter(audio_data, self.low_freq, self.high_freq, self.sample_rate)
        current_bpm = self.bpm_detector.update(filtered)
        
        # Publish BPM
        bpm_msg = Int32()
        bpm_msg.data = current_bpm
        self.bpm_publisher.publish(bpm_msg)
        
        # Update audio buffers
        self.audio_buffer = np.roll(self.audio_buffer, -len(audio_data))
        self.audio_buffer[-len(audio_data):] = audio_data
        
        self.fft_buffer = np.roll(self.fft_buffer, -len(audio_data))
        self.fft_buffer[-len(audio_data):] = audio_data
        
        return (in_data, pyaudio.paContinue)

    def start_audio(self):
        try:
            self.stream = self.p.open(
                format=pyaudio.paFloat32,
                channels=1,
                rate=self.sample_rate,
                input=True,
                frames_per_buffer=self.chunk_size,
                input_device_index=self.device_index,
                stream_callback=self._audio_callback
            )
            self.stream.start_stream()
            self.running = True
            
            while self.running and rclpy.ok():
                time.sleep(0.1)
                
        except Exception as e:
            self.get_logger().error(f'Audio stream error: {str(e)}')
        finally:
            if self.stream:
                self.stream.stop_stream()
                self.stream.close()
            self.p.terminate()

    def find_mic(self):
        for i in range(self.p.get_device_count()):
            info = self.p.get_device_info_by_index(i)
            if "USB Device" in info['name']:
                self.get_logger().info(f"Using audio device: {info['name']} (index {i})")
                return i
        raise RuntimeError("Logitech audio device not found!")

    # def generate_waveform_image(self):
    #     fig, ax = plt.subplots(figsize=(10, 4), dpi=100)
        
    #     filtered_data = self._apply_bandpass_filter(
    #         self.audio_buffer, self.low_freq, self.high_freq, self.sample_rate
    #     )
        
    #     ax.plot(self.time_buffer, self.audio_buffer, label='Raw Audio')
    #     ax.plot(self.time_buffer, filtered_data, label='Filtered Audio', color='r', alpha=0.6)
    #     ax.set_xlim([-self.buffer_seconds, 0])
    #     ax.set_ylim([-1, 1])
    #     ax.set_xlabel('Time (s)')
    #     ax.set_ylabel('Amplitude')
    #     ax.set_title(f'Audio Waveform — BPM: {self.bpm_detector.last_bpm}')
    #     ax.legend()
    #     ax.grid(True)
        
    #     fig.tight_layout()
        
    #     # Convert figure to image
    #     fig.canvas.draw()
    #     img = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
    #     img = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))
    #     plt.close(fig)
        
    #     return img

    # def generate_spectrum_image(self):
    #     fig, ax = plt.subplots(figsize=(10, 4), dpi=100)
        
    #     fft_data = np.abs(np.fft.rfft(self.fft_buffer)) / self.fft_size
        
    #     ax.plot(self.freq_buffer, fft_data)
    #     ax.set_xlim([0, 2000])
    #     ax.set_xlabel('Frequency (Hz)')
    #     ax.set_ylabel('Magnitude')
    #     ax.set_title('Frequency Spectrum')
    #     ax.axvspan(self.low_freq, self.high_freq, alpha=0.2, color='red')
    #     ax.grid(True)
        
    #     fig.tight_layout()
        
    #     # Convert figure to image
    #     fig.canvas.draw()
    #     img = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
    #     img = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))
    #     plt.close(fig)
        
    #     return img

    # def timer_callback(self):
    #     # Publish raw waveform data
    #     waveform_msg = Float32MultiArray()
    #     waveform_msg.data = self.audio_buffer.tolist()
    #     self.waveform_publisher.publish(waveform_msg)
        
    #     # Publish spectrum data
    #     fft_data = np.abs(np.fft.rfft(self.fft_buffer)) / self.fft_size
    #     spectrum_msg = Float32MultiArray()
    #     spectrum_msg.data = fft_data.tolist()
    #     self.spectrum_publisher.publish(spectrum_msg)
        
    #     # Generate and publish visualization images
    #     try:
    #         # Waveform image
    #         waveform_img = self.generate_waveform_image()
    #         waveform_img_msg = self.bridge.cv2_to_imgmsg(waveform_img, encoding="rgb8")
    #         self.waveform_img_publisher.publish(waveform_img_msg)
            
    #         # Spectrum image
    #         spectrum_img = self.generate_spectrum_image()
    #         spectrum_img_msg = self.bridge.cv2_to_imgmsg(spectrum_img, encoding="rgb8")
    #         self.spectrum_img_publisher.publish(spectrum_img_msg)
    #     except Exception as e:
    #         self.get_logger().error(f'Error generating visualization: {str(e)}')

    def destroy_node(self):
        self.get_logger().info('Shutting down BPM Detector Node')
        self.running = False
        if self.audio_thread.is_alive():
            self.audio_thread.join(timeout=1.0)
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
        self.p.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BPMDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
