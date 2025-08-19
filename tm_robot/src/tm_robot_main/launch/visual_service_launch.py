from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Main control node
        Node(
            package='visuial',
            executable='realsense',  
            name='realsense_node',
            output='screen',
            parameters=[],
        ),
        Node(
            package='visuial',
            executable='medicine_check_service',  
            name='drug_identify_service',
            output='screen',
            parameters=[],
        ),
        Node(
            package='visuial',
            executable='double_check', 
            name='camera_publisher',
            output='screen'
        ),
        Node(
            package='visuial',
            executable='second_camera', 
            name='second_camera',
            output='screen',
            parameters=[]
        ),
        Node(
            package='visuial',
            executable='run_paadleocr', 
            name='run_paadleocr',
            output='screen',
            parameters=[]
        )
    ])
