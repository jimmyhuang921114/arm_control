from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tm_robot_main',  
            executable='main_control', 
            name='main_control_node',
            output='screen',
            parameters=[],
            remappings=[
                # ('/input/image_raw', '/camera/color/image_raw'),
            ]
        )
        Node(
            package = 
        )
    ])
