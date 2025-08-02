from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Main control node
        Node(
            package='tm_robot_main',
            executable='main_control',  # 應確認這個是否正確（通常 executable 應是 'realsense'）
            name='main_control',
            output='screen',
            parameters=[],
        ),
        Node(
            package='tm_robot_main',
            executable='slider_simple',  # 應確認這個是否正確（通常 executable 應是 'realsense'）
            name='slider_simple',
            output='screen',
            parameters=[],
        ),
        Node(
            package='tm_robot_main',
            executable='order_report',  # 應確認這個是否正確（通常 executable 應是 'realsense'）
            name='order_report',
            output='screen',
            parameters=[],
        ),
    ])
