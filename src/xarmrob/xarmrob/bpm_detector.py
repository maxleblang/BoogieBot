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
        self.last_bpm = 80  # Initialize with a reasonable default
        self.recent_bpms = [80]  # Start with a reasonable value
        
        # Beat detection variables
        self.last_beat_time = time.time()  # Initialize to current time
        self.beat_threshold = 1.5  # Lower threshold for easier detection
        self.min_time_between_beats = 0.2  # Seconds, prevents too many false positives
        self.max_time_between_beats = 2.0  # Maximum time to wait for a beat
        self.beat_callback = None
        
        # Enhanced BPM stability
        self.bpm_history_size = 10  # Reduced for more responsiveness
        self.peak_intervals = []
        self.peak_interval_history_size = 20
        self.last_reported_bpm = 80  # Initialize with a reasonable default
        self.stability_threshold = 3  # BPM must change by at least this amount to update
        self.confidence_counter = 0
        self.min_confidence = 3  # Reduced for more responsiveness
        
        # Energy tracking for better peak detection
        self.energy_history = []
        self.energy_history_size = 30
        self.local_energy_window = 5
        
        # Debug flag
        self.debug = False

    def set_beat_callback(self, callback):
        self.beat_callback = callback
        
    def set_debug(self, debug):
        self.debug = debug

    def update(self, audio_chunk):
        # Calculate audio energy
        rectified = np.abs(audio_chunk)
        smoothed = lfilter([1], [1, -0.95], rectified)  # Less aggressive filter
        energy = np.mean(smoothed**2)  # Squared for better dynamics
        
        if self.debug and len(self.energy_history) % 50 == 0:
            print(f"Energy: {energy:.6f}")
        
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
            # Calculate dynamic threshold
            local_energy = np.mean(self.energy_history[-self.local_energy_window:])
            global_energy = np.mean(self.energy_history)
            global_std = np.std(self.energy_history) if len(self.energy_history) > 1 else 0.0001
            
            # Ensure we have a minimum threshold even when audio is quiet
            threshold = max(global_energy + self.beat_threshold * global_std, 0.0001)
            
            time_since_last_beat = current_time - self.last_beat_time
            expected_beat_time = 60.0 / max(self.last_bpm, 60) if self.last_bpm > 0 else 1.0
            
            # Only check for beats if we have sufficient energy
            if energy > threshold and time_since_last_beat > self.min_time_between_beats:
                beat_detected = True
                
                if self.debug:
                    print(f"Beat detected! Energy: {energy:.6f}, Threshold: {threshold:.6f}")
                
                # Record the beat interval
                if self.last_beat_time > 0:
                    interval = time_since_last_beat
                    if 0.2 < interval < 2.0:  # Sanity check: 30-300 BPM range
                        self.peak_intervals.append(interval)
                        if len(self.peak_intervals) > self.peak_interval_history_size:
                            self.peak_intervals.pop(0)
                
                self.last_beat_time = current_time
                
                # Trigger callback
                if self.beat_callback:
                    self.beat_callback(self.last_bpm)
            
            # Forced beat detection if we've waited too long
            elif time_since_last_beat > min(self.max_time_between_beats, expected_beat_time * 1.5):
                # Only force a beat if we have some energy increase
                if len(self.energy_history) >= 3 and energy > self.energy_history[-2]:
                    if self.debug:
                        print(f"Forced beat! Time since last: {time_since_last_beat:.2f}s")
                    
                    beat_detected = True
                    self.last_beat_time = current_time
                    
                    if self.beat_callback:
                        self.beat_callback(self.last_bpm)
        
        # Calculate BPM - simplify for now to get it working
        if len(self.envelope_buffer) >= 10:
            # Try both methods
            calculated_bpm = self.calculate_bpm()
            interval_bpm = self.calculate_interval_bpm() 
            
            if self.debug and (calculated_bpm > 0 or interval_bpm > 0):
                print(f"Calculated BPM: {calculated_bpm}, Interval BPM: {interval_bpm}")
            
            # Choose the best available BPM
            if calculated_bpm > 0 and interval_bpm > 0:
                # If both methods available, prefer interval BPM if we have enough data
                if len(self.peak_intervals) >= 4:
                    proposed_bpm = int(round(interval_bpm * 0.7 + calculated_bpm * 0.3))
                else:
                    proposed_bpm = calculated_bpm
            elif calculated_bpm > 0:
                proposed_bpm = calculated_bpm
            elif interval_bpm > 0:
                proposed_bpm = interval_bpm
            else:
                # Keep current BPM if no new data
                proposed_bpm = self.last_bpm
            
            # Apply simple BPM limiting
            if 40 <= proposed_bpm <= 200:
                # Simple stability rule
                if abs(proposed_bpm - self.last_reported_bpm) <= self.stability_threshold:
                    self.last_bpm = proposed_bpm
                    self.last_reported_bpm = proposed_bpm
                else:
                    # Only update if we detected an actual beat
                    if beat_detected:
                        self.confidence_counter += 1
                        if self.confidence_counter >= self.min_confidence:
                            if self.debug:
                                print(f"BPM changing from {self.last_bpm} to {proposed_bpm}")
                            self.last_bpm = proposed_bpm
                            self.last_reported_bpm = proposed_bpm
                            self.confidence_counter = 0
                    else:
                        self.confidence_counter = 0
                
        return self.last_bpm, beat_detected

    def calculate_bpm(self):
        """Simplified peak detection based BPM calculation"""
        if len(self.envelope_buffer) < 10:
            return self.last_bpm
            
        # Use the envelope for peak detection
        env = np.array(self.envelope_buffer)
        if np.std(env) > 0:
            env = (env - np.mean(env)) / (np.std(env) + 1e-6)  # Normalize
        else:
            # Not enough variation for beat detection
            return self.last_bpm

        # Find peaks in the envelope
        min_interval = int(0.3 * self.sample_rate / self.chunk_size)  # Minimum 0.3s between peaks (200 BPM max)
        try:
            peaks, _ = find_peaks(env, height=0.5, distance=min_interval)
        except Exception as e:
            if self.debug:
                print(f"Peak finding error: {str(e)}")
            return self.last_bpm

        if len(peaks) < 2:
            return self.last_bpm

        # Calculate intervals
        intervals = np.diff(peaks) * (self.chunk_size / self.sample_rate)
        if len(intervals) == 0:
            return self.last_bpm
        
        # Use median for better robustness
        median_interval = np.median(intervals)
        if median_interval <= 0.001:  # Avoid division by zero
            return self.last_bpm
            
        bpm = 60.0 / median_interval

        # Add to history
        self.recent_bpms.append(bpm)
        if len(self.recent_bpms) > self.bpm_history_size:
            self.recent_bpms.pop(0)

        # Use median for stability
        return int(round(np.median(self.recent_bpms)))
        
    def calculate_interval_bpm(self):
        """Calculate BPM directly from measured beat intervals"""
        if len(self.peak_intervals) < 3:
            return 0  # Not enough data
            
        # Simple median calculation - more robust than mean
        median_interval = np.median(self.peak_intervals)
        if median_interval <= 0.001:  # Avoid division by zero
            return 0
            
        bpm = 60.0 / median_interval
        
        # Sanity check
        if 40 <= bpm <= 200:
            return int(round(bpm))
        else:
            return 0


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
        self.declare_parameter('debug', False)  # Enable debug output
        
        self.sample_rate = self.get_parameter('sample_rate').value
        self.chunk_size = self.get_parameter('chunk_size').value
        self.buffer_seconds = self.get_parameter('buffer_seconds').value
        self.low_freq = self.get_parameter('low_freq').value
        self.high_freq = self.get_parameter('high_freq').value
        self.debug = self.get_parameter('debug').value
        
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
            
        # Enable debug if requested
        if self.debug:
            self.bpm_detector.set_debug(True)
            self.get_logger().info("Debug mode enabled for BPM detection")
        
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
        max_amplitude = np.max(np.abs(audio_data))
        if max_amplitude > 0.001:  # Only normalize if we have actual signal
            normalized = audio_data / max_amplitude
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
        
        # Check for valid audio data
        if np.isnan(filtered).any() or np.isinf(filtered).any():
            if self.debug:
                self.get_logger().warn("Invalid audio data detected - skipping frame")
            return (in_data, pyaudio.paContinue)
            
        # Apply additional envelope enhancement for beat detection
        filtered = np.abs(filtered)  # Full-wave rectification
        
        # Update detector and get BPM and beat status
        current_bpm, beat_detected = self.bpm_detector.update(filtered)
        
        if self.debug and beat_detected:
            self.get_logger().info(f"Beat detected with BPM: {current_bpm}")
        
        # Always publish BPM for now to help with debugging
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
        """Find the audio input device to use"""
        device_found = False
        default_device = None
        
        for i in range(self.p.get_device_count()):
            info = self.p.get_device_info_by_index(i)
            # Save the default input device as fallback
            if info['maxInputChannels'] > 0 and default_device is None:
                default_device = i
                
            self.get_logger().info(f"Audio device {i}: {info['name']} (in: {info['maxInputChannels']}, out: {info['maxOutputChannels']})")
            
            # Try to find the USB device
            if "USB" in info['name'] and info['maxInputChannels'] > 0:
                self.get_logger().info(f"Using audio device: {info['name']} (index {i})")
                return i
                
        # Use default input device if specified device not found
        if default_device is not None:
            dev_info = self.p.get_device_info_by_index(default_device)
            self.get_logger().warn(f"USB audio device not found! Using default input device: {dev_info['name']} (index {default_device})")
            return default_device
            
        self.get_logger().error("No suitable audio input device found!")
        return None

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