from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    """
    啟動增強型細小物件抓取系統
    包含視覺識別、智能分類和抓取控制
    """
    
    # 聲明參數
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('graspnet'),
            'graspnet',
            'small_object_config.yaml'
        ]),
        description='配置文件路徑'
    )
    
    debug_mode_arg = DeclareLaunchArgument(
        'debug_mode',
        default_value='false',
        description='是否啟用調試模式'
    )
    
    use_simulation_arg = DeclareLaunchArgument(
        'use_simulation',
        default_value='false',
        description='是否使用模擬環境'
    )
    
    # 增強型抓取檢測節點
    enhanced_grasp_node = Node(
        package='graspnet',
        executable='enhanced_small_object_grasp',
        name='enhanced_grasp_detector',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
            'debug_mode': LaunchConfiguration('debug_mode'),
            'use_rviz_visualization': True,
        }],
        remappings=[
            ('/tm_robot/color_image', '/camera/color/image_raw'),
            ('/tm_robot/depth_image', '/camera/depth/image_raw'),
            ('/enhanced_grasp_pose', '/robot/target_pose'),
        ]
    )
    
    # 相機驅動（如果不是模擬模式）
    camera_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense_camera',
        output='screen',
        parameters=[{
            'enable_color': True,
            'enable_depth': True,
            'color_width': 1280,
            'color_height': 720,
            'depth_width': 1280,
            'depth_height': 720,
            'color_fps': 30.0,
            'depth_fps': 30.0,
            'align_depth': True,
        }],
        condition=lambda context: LaunchConfiguration('use_simulation').perform(context) == 'false'
    )
    
    # RViz可視化
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('graspnet'),
        'config',
        'enhanced_grasp_visualization.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=lambda context: LaunchConfiguration('debug_mode').perform(context) == 'true'
    )
    
    # TF發布器（靜態變換）
    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_base_tf',
        arguments=[
            '0', '0', '0.5',  # x, y, z
            '0', '0', '0', '1',  # qx, qy, qz, qw
            'base_link',
            'camera_link'
        ]
    )
    
    # 診斷節點（可選）
    diagnostic_node = Node(
        package='graspnet',
        executable='grasp_system_monitor',
        name='grasp_system_monitor',
        output='screen',
        parameters=[{
            'monitor_frequency': 10.0,
            'performance_logging': True,
        }],
        condition=lambda context: LaunchConfiguration('debug_mode').perform(context) == 'true'
    )

    return LaunchDescription([
        # 參數聲明
        config_file_arg,
        debug_mode_arg,
        use_simulation_arg,
        
        # 核心節點
        enhanced_grasp_node,
        static_tf_publisher,
        
        # 可選節點
        camera_node,
        rviz_node,
        diagnostic_node,
    ])