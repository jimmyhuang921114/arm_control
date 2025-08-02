from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Main control node
        Node(
            package='locate',
            executable='shelf',  # 應確認這個是否正確（通常 executable 應是 'realsense'）
            name='shelf',
            output='screen',
            parameters=[],
        ),
        Node(
            package='locate',
            executable='slider',  # 應確認這個是否正確（通常 executable 應是 'realsense'）
            name='slider',
            output='screen',
            parameters=[],
        )
    ])
