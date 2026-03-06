from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='eced3901',
            executable='tof_serial_bridge.py',
            name='tof_serial_bridge',
            output='screen'
        ),
        Node(
            package='eced3901',
            executable='align_parallel_parking',
            name='align_parallel_parking',
            output='screen'
        )
    ])
