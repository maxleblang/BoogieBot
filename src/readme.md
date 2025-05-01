# BPM Detector ROS2 Package

A ROS2 package for real-time audio BPM (Beats Per Minute) detection running on Raspberry Pi.

## Overview

This package provides a ROS2 node that:
- Captures audio from a microphone
- Performs real-time BPM detection using signal processing
- Publishes BPM data and visualizations to ROS2 topics
- Optimized for running on Raspberry Pi

## Features

- **Real-time BPM detection**: Detects BPM in the range of 40-200 BPM
- **Audio filtering**: Bandpass filtering to focus on beat frequencies
- **Visualization**: Waveform and spectrum visualizations
- **Resource-efficient**: Optimized for Raspberry Pi's limited resources
- **Configurable parameters**: Sample rate, frequency ranges, etc.

## Topics

The node publishes the following topics:

| Topic | Type | Description |
|-------|------|-------------|
| `/bpm` | `std_msgs/Int32` | Current detected BPM |
| `/audio_waveform` | `std_msgs/Float32MultiArray` | Raw audio waveform data |
| `/audio_spectrum` | `std_msgs/Float32MultiArray` | Audio spectrum data |
| `/waveform_image` | `sensor_msgs/Image` | Visualization of audio waveform |
| `/spectrum_image` | `sensor_msgs/Image` | Visualization of frequency spectrum |

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `sample_rate` | int | 44100 | Audio sample rate in Hz |
| `chunk_size` | int | 1024 | Audio processing chunk size |
| `buffer_seconds` | int | 5 | Audio buffer duration in seconds |
| `low_freq` | int | 60 | Low cutoff frequency for bandpass filter |
| `high_freq` | int | 250 | High cutoff frequency for bandpass filter |
| `device_index` | int | None | Audio input device index (None = default) |
| `visualization_rate` | float | 5.0 | Rate to publish visualizations (Hz) |

## Nodes

### 1. `bpm_detector_node`

The main node for processing audio and detecting BPM.

**Published Topics:**
- All topics listed in the Topics section above

**Parameters:**
- All parameters listed in the Parameters section above

### 2. `bpm_visualization_node` (Optional)

A node for displaying the BPM and audio visualizations on a connected monitor.

**Subscribed Topics:**
- `/bpm` (Int32): Current detected BPM value
- `/waveform_image` (Image): Waveform visualization 
- `/spectrum_image` (Image): Spectrum visualization

## Installation

See the [Installation Guide](INSTALL.md) for detailed setup instructions.

## Usage

### Basic launch
```bash
ros2 launch bpm_detector bpm_detector.launch.py
```

### Launch with custom parameters
```bash
ros2 launch bpm_detector bpm_detector.launch.py sample_rate:=22050 low_freq:=80 high_freq:=200 visualization_rate:=2.0
```

### Monitor BPM
```bash
ros2 topic echo /bpm
```

## Comparison with Original Code

This package is a ROS2 reimplementation of a standalone audio BPM detector with the following improvements:

- Structured as a proper ROS2 package with nodes, topics, and parameters
- Decoupled audio processing from visualization
- Added ROS2 messages for data communication
- Added configuration through ROS2 parameters
- Optimized for Raspberry Pi's limited resources
- Made visualization optional to reduce resource usage
- Added launch files for easy deployment

## License

This package is licensed under the Apache License 2.0.

## Author

Your Name (<your_email@example.com>)