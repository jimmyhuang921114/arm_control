from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Main control node
        Node(
            package='tm_robot_main',
            executable='main_control', 
            name='main_control',
            output='screen',
            parameters=[],
        ),
        Node(
            package='locate',
            executable='fix_shelf',  
            name='fix_shelf',
            output='screen',
            parameters=[],
        ),
        Node(
            package='locate',
            executable='fix_slider',  
            name='fix_slider',
            output='screen',
            parameters=[],
        ),
        Node(
            package='tm_robot_main',
            executable='order_process',  
            name='order_process',
            output='screen',
            parameters=[],
        ),
        Node(
            package='tm_robot_main',
            executable='tm_flow_mode',
            name='tm_flow_mode',
            output = 'screen',
            parameters=[]
        )
    ])
