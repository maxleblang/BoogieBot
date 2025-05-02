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
    def __init__(self, sample_rate=44100, chunk_size=1024, history_sec=10):
        self.sample_rate = sample_rate
        self.chunk_size = chunk_size
        self.buffer_len = int(sample_rate / chunk_size * history_sec)
        self.envelope_buffer = []
        self.last_bpm = 0
        self.recent_bpms = []
        
        # Beat detection variables
        self.last_beat_time = 0
        self.beat_threshold = 1.8  # Adjust as needed
        self.min_time_between_beats = 0.2  # Seconds, prevents too many false positives
        self.max_time_between_beats = 2.0  # Maximum time to wait for a beat
        self.beat_callback = None
        
        # Enhanced BPM stability
        self.bpm_history_size = 20  # Longer history for more stability
        self.peak_intervals = []
        self.peak_interval_history_size = 30
        self.last_reported_bpm = 0
        self.stability_threshold = 3  # BPM must change by at least this amount to update
        self.confidence_counter = 0
        self.min_confidence = 5  # Minimum number of consistent readings before changing BPM
        
        # Energy tracking for better peak detection
        self.energy_history = []
        self.energy_history_size = 30
        self.local_energy_window = 5

    def set_beat_callback(self, callback):
        self.beat_callback = callback

    def update(self, audio_chunk):
        # Calculate audio energy (enhanced from simple mean)
        rectified = np.abs(audio_chunk)
        # Use a better filter for smoothing (less aggressive decay)
        smoothed = lfilter([1], [1, -0.98], rectified)
        energy = np.mean(smoothed**2)  # Squared for better dynamics
        
        # Track energy history for adaptive thresholding
        self.energy_history.append(energy)
        if len(self.energy_history) > self.energy_history_size:
            self.energy_history.pop(0)
        
        # Store in envelope buffer for BPM calculation
        self.envelope_buffer.append(energy)
        if len(self.envelope_buffer) > self.buffer_len:
            self.envelope_buffer.pop(0)
        
        # Beat detection with improved threshold calculation
        beat_detected = False
        current_time = time.time()
        
        if len(self.energy_history) >= self.local_energy_window:
            # Use a local vs global energy comparison for better adaptability
            local_energy = np.mean(self.energy_history[-self.local_energy_window:])
            global_energy = np.mean(self.energy_history)
            global_std = np.std(self.energy_history)
            
            # Dynamic threshold that adapts to both overall volume and recent changes
            threshold = global_energy + self.beat_threshold * global_std
            
            # Beat detection with additional conditions
            time_since_last_beat = current_time - self.last_beat_time
            expected_beat_time = 60.0 / max(self.last_bpm, 60) if self.last_bpm > 0 else 1.0
            
            # Require stronger onset if we're early in the beat cycle
            # and allow weaker onset if we're late in the beat cycle
            beat_cycle_position = time_since_last_beat / expected_beat_time
            adaptive_threshold = threshold * (1.2 - min(0.4, beat_cycle_position * 0.5))
            
            # Primary condition: current energy spike exceeds threshold
            if energy > adaptive_threshold:
                # Minimum time constraint to avoid double-triggering
                if time_since_last_beat > self.min_time_between_beats:
                    # Check that this is actually a local peak
                    if len(self.energy_history) >= 3 and energy > self.energy_history[-2]:
                        beat_detected = True
                        self.last_beat_time = current_time
                        
                        # Record interval for BPM calculation if we have a previous beat
                        if self.last_bpm > 0:
                            interval = time_since_last_beat
                            if 0.1 < interval < 2.0:  # Sanity check: 30-600 BPM range
                                self.peak_intervals.append(interval)
                                if len(self.peak_intervals) > self.peak_interval_history_size:
                                    self.peak_intervals.pop(0)
                        
                        # Trigger callback
                        if self.beat_callback:
                            self.beat_callback(self.last_bpm)
            
            # Forced beat detection if we've waited too long
            # This helps maintain continuity when beats are occasionally missed
            elif time_since_last_beat > min(self.max_time_between_beats, expected_beat_time * 1.5):
                # Only trigger if we have a reasonable energy increase recently
                recent_energies = self.energy_history[-5:]
                if len(recent_energies) >= 5 and energy > np.mean(recent_energies[:-1]):
                    beat_detected = True
                    self.last_beat_time = current_time
                    
                    if self.beat_callback:
                        self.beat_callback(self.last_bpm)
            
        # Calculate and update BPM
        if len(self.envelope_buffer) >= self.buffer_len // 2:
            # Use both peak detection algorithm and direct beat interval measurement
            calculated_bpm = self.calculate_bpm()
            interval_bpm = self.calculate_interval_bpm()
            
            # Combine both methods, favoring the more stable one
            if interval_bpm > 0 and calculated_bpm > 0:
                # Weighted average, with more weight to the interval method if we have enough data
                weight = min(0.8, len(self.peak_intervals) / self.peak_interval_history_size)
                combined_bpm = interval_bpm * weight + calculated_bpm * (1 - weight)
                proposed_bpm = int(round(combined_bpm))
            elif interval_bpm > 0:
                proposed_bpm = interval_bpm
            elif calculated_bpm > 0:
                proposed_bpm = calculated_bpm
            else:
                proposed_bpm = self.last_bpm
            
            # Apply stability rules to avoid jumps
            if abs(proposed_bpm - self.last_reported_bpm) <= self.stability_threshold:
                # BPM is stable, increase confidence
                self.confidence_counter += 1
                if self.confidence_counter >= self.min_confidence:
                    # We've had stable readings long enough, update the BPM
                    self.last_bpm = proposed_bpm
                    self.last_reported_bpm = proposed_bpm
            else:
                # Large change detected, reset confidence but don't change BPM yet
                self.confidence_counter = 0
                # If change persists for multiple frames, eventually accept it
                if self.confidence_counter <= -3:
                    self.last_bpm = proposed_bpm
                    self.last_reported_bpm = proposed_bpm
                    self.confidence_counter = 0
                else:
                    self.confidence_counter -= 1
                
        return self.last_bpm, beat_detected

    def calculate_bpm(self):
        """Peak detection based BPM calculation"""
        if len(self.envelope_buffer) < 16:
            return self.last_bpm
            
        # Use the envelope for peak detection
        env = np.array(self.envelope_buffer)
        env = (env - np.mean(env)) / (np.std(env) + 1e-6)  # Normalize

        # More aggressive minimum distance between peaks for stability
        min_interval = int(0.4 * self.sample_rate / self.chunk_size)
        peaks, properties = find_peaks(env, height=1.3, distance=min_interval, prominence=0.5)

        if len(peaks) < 3:  # Require more peaks for better accuracy
            return self.last_bpm

        # Calculate intervals and filter outliers
        intervals = np.diff(peaks) * (self.chunk_size / self.sample_rate)
        if len(intervals) == 0:
            return self.last_bpm
            
        # Filter out extreme values
        valid_intervals = intervals[(intervals > 0.3) & (intervals < 2.0)]
        if len(valid_intervals) < 2:
            return self.last_bpm
            
        # Use median for better robustness to outliers
        median_interval = np.median(valid_intervals)
        if median_interval == 0:
            return self.last_bpm
            
        bpm = 60.0 / median_interval

        # Add to history
        self.recent_bpms.append(bpm)
        if len(self.recent_bpms) > self.bpm_history_size:
            self.recent_bpms.pop(0)

        # Use a weighted median - favor more recent BPM calculations
        weights = np.linspace(0.5, 1.0, len(self.recent_bpms))
        weighted_bpms = np.array(self.recent_bpms) * weights
        
        # Return median of weighted values for stability
        return int(round(np.median(weighted_bpms)))
        
    def calculate_interval_bpm(self):
        """Calculate BPM directly from measured beat intervals"""
        if len(self.peak_intervals) < 4:
            return 0  # Not enough data
            
        # Filter intervals to remove outliers
        valid_intervals = np.array(self.peak_intervals)
        median_interval = np.median(valid_intervals)
        
        # Keep only intervals within 30% of the median
        filtered_intervals = valid_intervals[np.abs(valid_intervals - median_interval) < (0.3 * median_interval)]
        
        if len(filtered_intervals) < 3:
            return 0  # Not enough consistent intervals
            
        # Calculate BPM from average interval
        average_interval = np.mean(filtered_intervals)
        return int(round(60.0 / average_interval))


class BPMDetectorNode(Node):
    def __init__(self):
        super().__init__('bpm_detector_node')
        
        # Parameters
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('chunk_size', 1024)
        self.declare_parameter('buffer_seconds', 10)
        self.declare_parameter('low_freq', 40)  # Lower frequency to catch more bass content
        self.declare_parameter('high_freq', 300)  # Wider range for better music detection
        self.declare_parameter('device_index', None)
        self.declare_parameter('visualization_rate', 5.0)  # Hz
        self.declare_parameter('bpm_stability_threshold', 3)  # BPM must change by this amount to update
        
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
        
        # Create BPM detector with enhanced stability
        self.bpm_detector = BPMDetector(
            sample_rate=self.sample_rate,
            chunk_size=self.chunk_size,
            history_sec=10  # Longer history for more stable BPM calculation
        )
        
        # Configure stability if parameter was provided
        stability_threshold = self.get_parameter('bpm_stability_threshold').value
        if stability_threshold:
            self.bpm_detector.stability_threshold = stability_threshold
        
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
        # Convert audio data and apply better filtering
        audio_data = np.frombuffer(in_data, dtype=np.float32)
        
        # Apply normalization to reduce effects of volume changes
        if np.max(np.abs(audio_data)) > 0:
            normalized = audio_data / (np.max(np.abs(audio_data)) + 1e-6)
        else:
            normalized = audio_data
            
        # Apply bandpass filter with steeper cutoffs (higher order filter)
        filtered = self._apply_bandpass_filter(
            normalized, 
            self.low_freq, 
            self.high_freq, 
            self.sample_rate,
            order=4  # Higher order for steeper cutoff
        )
        
        # Apply additional envelope enhancement for beat detection
        filtered = np.abs(filtered)  # Full-wave rectification
        
        # Update detector and get BPM and beat status
        current_bpm, beat_detected = self.bpm_detector.update(filtered)
        
        # Throttle BPM publishing to reduce noise
        # Only publish when beat is detected or when BPM changes
        if beat_detected or getattr(self, 'last_published_bpm', 0) != current_bpm:
            bpm_msg = Int32()
            bpm_msg.data = current_bpm
            self.bpm_publisher.publish(bpm_msg)
            self.last_published_bpm = current_bpm
        
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