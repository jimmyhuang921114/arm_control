#!/usr/bin/env python3
"""
完整藥局系統啟動腳本
統一啟動所有子系統：視覺識別、抓取檢測、系統協調、用戶界面等
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, GroupAction,
    IncludeLaunchDescription, TimerAction
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
import os

def generate_launch_description():
    """
    生成完整藥局系統啟動描述
    按順序啟動各個子系統，確保依賴關係正確
    """
    
    # 聲明啟動參數
    use_simulation_arg = DeclareLaunchArgument(
        'use_simulation',
        default_value='false',
        description='是否使用模擬環境'
    )
    
    debug_mode_arg = DeclareLaunchArgument(
        'debug_mode',
        default_value='false',
        description='是否啟用調試模式'
    )
    
    enable_gui_arg = DeclareLaunchArgument(
        'enable_gui',
        default_value='true',
        description='是否啟用圖形界面'
    )
    
    enable_monitoring_arg = DeclareLaunchArgument(
        'enable_monitoring',
        default_value='true',
        description='是否啟用系統監控'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='',
        description='自定義配置文件路徑'
    )
    
    # ===== 核心系統組件 =====
    
    # 1. 系統總協調器（最高優先級）
    system_orchestrator = Node(
        package='tm_robot_main',
        executable='pharmacy_system_orchestrator',
        name='pharmacy_system_orchestrator',
        output='screen',
        parameters=[{
            'debug_mode': LaunchConfiguration('debug_mode'),
            'config_file': LaunchConfiguration('config_file'),
        }],
        respawn=True,
        respawn_delay=5.0
    )
    
    # 2. 視覺識別系統
    vision_system_group = GroupAction([
        # GroundedSAM2服務
        Node(
            package='groundedsam2',
            executable='grounded_sam2_service',
            name='grounded_sam2_service',
            output='screen',
            parameters=[{
                'model_config': 'configs/sam2.1/sam2.1_hiera_l.yaml',
                'checkpoint_path': './Grounded-SAM-2/checkpoints/sam2.1_hiera_large.pt'
            }]
        ),
        
        # 整合視覺識別系統
        TimerAction(
            period=3.0,  # 等待GroundedSAM2啟動
            actions=[
                Node(
                    package='tm_robot_main',
                    executable='integrated_vision_system',
                    name='integrated_vision_system',
                    output='screen',
                    parameters=[{
                        'debug_mode': LaunchConfiguration('debug_mode')
                    }]
                )
            ]
        ),
    ])
    
    # 3. 增強抓取檢測系統
    grasp_system_group = GroupAction([
        # 增強抓取檢測節點
        Node(
            package='graspnet',
            executable='enhanced_small_object_grasp',
            name='enhanced_grasp_detector',
            output='screen',
            parameters=[{
                'debug_mode': LaunchConfiguration('debug_mode')
            }]
        ),
        
        # 原有抓取檢測（備用）
        Node(
            package='graspnet',
            executable='grab_detect',
            name='grab_detector_backup',
            output='screen'
        )
    ])
    
    # 4. 機械手臂控制系統
    robot_control_group = GroupAction([
        # 主控制節點
        Node(
            package='tm_robot_main',
            executable='main_control',
            name='main_control',
            output='screen',
            parameters=[{
                'debug_mode': LaunchConfiguration('debug_mode')
            }]
        ),
        
        # TM機器人接口
        Node(
            package='tm_robot_if',
            executable='tm_robot_interface',
            name='tm_robot_interface',
            output='screen'
        )
    ])
    
    # 5. 用戶界面系統
    user_interface_group = GroupAction([
        # Web界面服務器
        ExecuteProcess(
            cmd=['python3', '-m', 'uvicorn', 'main:app', 
                 '--host', '0.0.0.0', '--port', '8000'],
            cwd='/workspace/tm_robot/src/user_interface',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_gui'))
        )
    ])
    
    # ===== 支援系統組件 =====
    
    # 6. 相機系統（如果不是模擬模式）
    camera_group = GroupAction([
        Node(
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
            condition=UnlessCondition(LaunchConfiguration('use_simulation'))
        )
    ])
    
    # 7. TF變換發布
    tf_publishers = GroupAction([
        # 相機到基座的靜態變換
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_base_tf',
            arguments=[
                '0', '0', '0.5',      # x, y, z
                '0', '0', '0', '1',   # qx, qy, qz, qw
                'base_link',
                'camera_link'
            ]
        ),
        
        # 機械手臂相關變換
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_robot_tf',
            arguments=[
                '0', '0', '0',
                '0', '0', '0', '1',
                'world',
                'base_link'
            ]
        )
    ])
    
    # 8. 監控和診斷系統
    monitoring_group = GroupAction([
        # 系統監控節點
        Node(
            package='tm_robot_main',
            executable='system_monitor',
            name='system_monitor',
            output='screen',
            parameters=[{
                'monitoring_frequency': 10.0,
                'alert_threshold': 0.8,
                'enable_performance_logging': True
            }],
            condition=IfCondition(LaunchConfiguration('enable_monitoring'))
        ),
        
        # 診斷聚合器
        Node(
            package='diagnostic_aggregator',
            executable='aggregator_node',
            name='diagnostic_aggregator',
            output='screen',
            parameters=[{
                'analyzers': {
                    'system': {
                        'type': 'diagnostic_aggregator/GenericAnalyzer',
                        'path': 'System',
                        'contains': ['pharmacy', 'vision', 'grasp']
                    }
                }
            }],
            condition=IfCondition(LaunchConfiguration('enable_monitoring'))
        )
    ])
    
    # 9. RViz可視化（調試模式）
    rviz_group = GroupAction([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('tm_robot_main'),
                'config',
                'pharmacy_system.rviz'
            ])],
            output='screen',
            condition=IfCondition(LaunchConfiguration('debug_mode'))
        )
    ])
    
    # 10. LLM藥物識別系統（二次確認用）
    llm_system_group = GroupAction([
        # LLM服務節點
        Node(
            package='dis_service',
            executable='llm_dis',
            name='llm_drug_identification',
            output='screen',
            cwd='/workspace/extra_package/llm_drug_identification_system/ros2_ws'
        )
    ])
    
    # ===== 啟動順序控制 =====
    
    # 第一階段：基礎系統（0秒）
    stage_1_basic_systems = GroupAction([
        tf_publishers,
        camera_group,
    ])
    
    # 第二階段：核心服務（3秒後）
    stage_2_core_services = TimerAction(
        period=3.0,
        actions=[
            vision_system_group,
            llm_system_group,
        ]
    )
    
    # 第三階段：抓取和控制系統（6秒後）
    stage_3_control_systems = TimerAction(
        period=6.0,
        actions=[
            grasp_system_group,
            robot_control_group,
        ]
    )
    
    # 第四階段：協調器（9秒後）
    stage_4_orchestrator = TimerAction(
        period=9.0,
        actions=[system_orchestrator]
    )
    
    # 第五階段：用戶界面和監控（12秒後）
    stage_5_ui_monitoring = TimerAction(
        period=12.0,
        actions=[
            user_interface_group,
            monitoring_group,
            rviz_group,
        ]
    )
    
    # ===== 啟動後處理 =====
    
    # 系統初始化完成通知
    initialization_complete = TimerAction(
        period=15.0,
        actions=[
            ExecuteProcess(
                cmd=['echo', '========================================'],
                output='screen'
            ),
            ExecuteProcess(
                cmd=['echo', '藥局系統啟動完成！'],
                output='screen'
            ),
            ExecuteProcess(
                cmd=['echo', 'Web界面: http://localhost:8000'],
                output='screen',
                condition=IfCondition(LaunchConfiguration('enable_gui'))
            ),
            ExecuteProcess(
                cmd=['echo', '系統監控: /pharmacy/system_status'],
                output='screen'
            ),
            ExecuteProcess(
                cmd=['echo', '========================================'],
                output='screen'
            ),
        ]
    )
    
    return LaunchDescription([
        # 參數聲明
        use_simulation_arg,
        debug_mode_arg,
        enable_gui_arg,
        enable_monitoring_arg,
        config_file_arg,
        
        # 分階段啟動
        stage_1_basic_systems,
        stage_2_core_services,
        stage_3_control_systems,
        stage_4_orchestrator,
        stage_5_ui_monitoring,
        
        # 完成通知
        initialization_complete,
    ])