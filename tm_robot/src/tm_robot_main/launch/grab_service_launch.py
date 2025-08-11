from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Main control node
        Node(
            package='graspnet',
            executable='grab_detect',
            name='plane_fitting_node',
            output='screen',
            parameters=[],
        ),

        # Second camera node (C++)
        Node(
            package='graspnet',
            executable='camera2base',
            name='camera2base',
            output='screen',
        ),                                         
    ])
