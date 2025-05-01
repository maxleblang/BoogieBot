from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    sample_rate_arg = DeclareLaunchArgument(
        'sample_rate',
        default_value='44100',
        description='Audio sample rate'
    )
    
    chunk_size_arg = DeclareLaunchArgument(
        'chunk_size',
        default_value='1024',
        description='Audio chunk size'
    )
    
    buffer_seconds_arg = DeclareLaunchArgument(
        'buffer_seconds',
        default_value='5',
        description='Audio buffer duration in seconds'
    )
    
    low_freq_arg = DeclareLaunchArgument(
        'low_freq',
        default_value='60',
        description='Low frequency cutoff for bandpass filter'
    )
    
    high_freq_arg = DeclareLaunchArgument(
        'high_freq',
        default_value='250',
        description='High frequency cutoff for bandpass filter'
    )
    
    visualization_rate_arg = DeclareLaunchArgument(
        'visualization_rate',
        default_value='5.0',
        description='Rate at which to publish visualization data (Hz)'
    )
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(sample_rate_arg)
    ld.add_action(chunk_size_arg)
    ld.add_action(buffer_seconds_arg)
    ld.add_action(low_freq_arg)
    ld.add_action(high_freq_arg)
    ld.add_action(visualization_rate_arg)
    
    # Add the BPM detector node
    bpm_detector_node = Node(
        package='bpm_detector',
        executable='bpm_detector_node',
        name='bpm_detector',
        parameters=[{
            'sample_rate': LaunchConfiguration('sample_rate'),
            'chunk_size': LaunchConfiguration('chunk_size'),
            'buffer_seconds': LaunchConfiguration('buffer_seconds'),
            'low_freq': LaunchConfiguration('low_freq'),
            'high_freq': LaunchConfiguration('high_freq'),
            'visualization_rate': LaunchConfiguration('visualization_rate')
        }],
        output='screen'
    )
    
    ld.add_action(bpm_detector_node)
    
    return ld
