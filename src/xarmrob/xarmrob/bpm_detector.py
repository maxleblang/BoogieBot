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
from std_msgs.msg import Int32, Empty


class BPMDetector:
    def __init__(self, sample_rate=44100, chunk_size=1024, history_sec=6):
        self.sample_rate = sample_rate
        self.chunk_size = chunk_size
        self.buffer_len = int(sample_rate / chunk_size * history_sec)
        self.envelope_buffer = []
        self.last_bpm = 120
        self.recent_bpms = []
        
        # Beat detection variables
        self.last_beat_time = 0
        self.beat_threshold = 1.5  # Adjust as needed
        self.min_time_between_beats = 0.3  # Seconds, prevents too many false positives
        self.beat_callback = None

    def set_beat_callback(self, callback):
        self.beat_callback = callback

    def update(self, audio_chunk):
        rectified = np.abs(audio_chunk)
        smoothed = lfilter([1], [1, -0.997], rectified)
        mean_env = np.mean(smoothed)
        self.envelope_buffer.append(mean_env)
        
        # Check if this is a beat
        beat_detected = False
        current_time = time.time()
        
        if len(self.envelope_buffer) > 5:  # Need some history for comparison
            # Calculate a dynamic threshold based on recent history
            recent_history = self.envelope_buffer[-20:]
            threshold = np.mean(recent_history) + self.beat_threshold * np.std(recent_history)
            
            # Check if current envelope exceeds threshold and enough time has passed
            if (mean_env > threshold and 
                current_time - self.last_beat_time > self.min_time_between_beats):
                beat_detected = True
                self.last_beat_time = current_time
                
                # Trigger callback
                if self.beat_callback:
                    self.beat_callback(self.last_bpm)
        
        # Maintain buffer size
        if len(self.envelope_buffer) > self.buffer_len:
            self.envelope_buffer.pop(0)
            
        # Calculate BPM
        if len(self.envelope_buffer) >= 16:
            bpm = self.calculate_bpm()
            if 120 <= bpm <= 122:
                self.last_bpm = bpm
                
        return self.last_bpm, beat_detected

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
        self.declare_parameter('sample_rate', 48000)
        self.declare_parameter('chunk_size', 1024)
        self.declare_parameter('buffer_seconds', 5)
        self.declare_parameter('low_freq', 50)
        self.declare_parameter('high_freq', 80)
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
        
        # Set beat callback
        self.bpm_detector.set_beat_callback(self.on_beat_detected)
        
        # Publishers
        self.bpm_publisher = self.create_publisher(Int32, 'bpm', 10)
        self.beat_publisher = self.create_publisher(Int32, 'beat', 10)  # Publishes BPM on each beat
        self.beat_event_publisher = self.create_publisher(Empty, 'beat_event', 10)  # Simple beat trigger
        
        # Initialize PyAudio
        self.p = pyaudio.PyAudio()
        self.stream = None
        self.running = False
        
        self.device_index = self.find_mic()
        
        # Start audio processing in a separate thread
        self.audio_thread = threading.Thread(target=self.start_audio)
        self.audio_thread.daemon = True
        self.audio_thread.start()
        
        self.get_logger().info('BPM Detector Node started')

    def on_beat_detected(self, current_bpm):
        """Callback when a beat is detected"""
        # Publish BPM on beat
        bpm_msg = Int32()
        bpm_msg.data = current_bpm
        self.beat_publisher.publish(bpm_msg)
        
        # Publish beat event
        self.beat_event_publisher.publish(Empty())
        
        self.get_logger().debug(f'Beat detected! Current BPM: {current_bpm}')

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
        
        # Update detector and get BPM and beat status
        current_bpm, beat_detected = self.bpm_detector.update(filtered)
        
        # Always publish current BPM (regardless of beat)
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
        self.get_logger().warning("Logitech audio device not found! Using default input device.")
        return None  # Use default input device

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