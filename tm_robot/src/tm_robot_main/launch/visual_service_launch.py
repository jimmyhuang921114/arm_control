from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Main control node
        Node(
            package='visuial',
            executable='realsense',  # 應確認這個是否正確（通常 executable 應是 'realsense'）
            name='realsense_node',
            output='screen',
            parameters=[],
        ),
        Node(
            package='visuial',
            executable='medicine_check_service',  # 應確認是否正確
            name='drug_identify_service',
            output='screen',
            parameters=[],
        ),
        Node(
            package='visuial',
            executable='double_check',  # 應確認是否正確
            name='camera_publisher',
            output='screen'
        ),
        Node(
            package='visuial',
            executable='paddleocr',  # 應確認是否正確
            name='easyocr_topic_node',
            output='screen',
            parameters=[]
        ),
    ])
