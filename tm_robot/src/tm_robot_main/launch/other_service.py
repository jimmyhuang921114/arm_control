from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Main control node
        Node(
            package='tm_robot_main',
            executable='main_control',
            name='main_control_node',
            output='screen',
            parameters=[],
        ),

        # Second camera node (C++)
        Node(
            package='second_camera',
            executable='second_camera',
            name='second_camera_node',
            output='screen',
        ),

        # RealSense image + point cloud publisher
        Node(
            package='visuial',
            executable='realsense',
            name='realsense_node',
            output='screen',
        ),

        # PaddleOCR node
        Node(
            package='visuial',
            executable='paddleocr',
            name='paddleocr_node',
            output='screen',
        ),

        # Medicine check service
        Node(
            package='visuial',
            executable='medicine_check_service',
            name='medicine_check_node',
            output='screen',
        ),
    ])
