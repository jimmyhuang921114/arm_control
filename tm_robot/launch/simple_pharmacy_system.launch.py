#!/usr/bin/env python3
"""
簡化版藥局系統啟動腳本
- GroundedSAM2用於物品挑選
- 增強抓取系統
- 網頁界面
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, GroupAction, TimerAction
)
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
import os

def generate_launch_description():
    """生成簡化版藥局系統啟動描述"""
    
    # 參數聲明
    use_simulation_arg = DeclareLaunchArgument(
        'use_simulation',
        default_value='false',
        description='是否使用模擬環境'
    )
    
    enable_gui_arg = DeclareLaunchArgument(
        'enable_gui',
        default_value='true',
        description='是否啟用網頁界面'
    )
    
    # 1. GroundedSAM2視覺識別服務
    grounded_sam2_service = Node(
        package='groundedsam2',
        executable='grounded_sam2_service',
        name='grounded_sam2_service',
        output='screen',
        parameters=[{
            'model_config': 'configs/sam2.1/sam2.1_hiera_l.yaml',
            'checkpoint_path': './Grounded-SAM-2/checkpoints/sam2.1_hiera_large.pt'
        }]
    )
    
    # 2. 增強抓取檢測系統
    enhanced_grasp_node = TimerAction(
        period=3.0,  # 等待GroundedSAM2啟動
        actions=[
            Node(
                package='graspnet',
                executable='enhanced_small_object_grasp',
                name='enhanced_grasp_detector',
                output='screen'
            )
        ]
    )
    
    # 3. 主控制系統
    main_control_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='tm_robot_main',
                executable='main_control',
                name='main_control',
                output='screen'
            )
        ]
    )
    
    # 4. 簡化藥局系統
    simple_pharmacy_system = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='tm_robot_main',
                executable='simple_pharmacy_system',
                name='simple_pharmacy_system',
                output='screen'
            )
        ]
    )
    
    # 5. 網頁界面
    web_interface = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=['python3', '-m', 'uvicorn', 'main:app', 
                     '--host', '0.0.0.0', '--port', '8000'],
                cwd='/workspace/tm_robot/src/user_interface',
                output='screen',
                condition=IfCondition(LaunchConfiguration('enable_gui'))
            )
        ]
    )
    
    # 6. 相機系統（真實環境）
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
        condition=UnlessCondition(LaunchConfiguration('use_simulation'))
    )
    
    # 7. TF變換
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_base_tf',
        arguments=[
            '0', '0', '0.5',      # x, y, z
            '0', '0', '0', '1',   # qx, qy, qz, qw
            'base_link',
            'camera_link'
        ]
    )
    
    # 8. LLM藥物識別系統（用於特定位置確認）
    llm_service = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='dis_service',
                executable='llm_dis',
                name='llm_drug_identification',
                output='screen',
                cwd='/workspace/extra_package/llm_drug_identification_system/ros2_ws'
            )
        ]
    )
    
    # 啟動完成通知
    startup_complete = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=['echo', '=============================='],
                output='screen'
            ),
            ExecuteProcess(
                cmd=['echo', '簡化藥局系統啟動完成！'],
                output='screen'
            ),
            ExecuteProcess(
                cmd=['echo', '工作流程:'],
                output='screen'
            ),
            ExecuteProcess(
                cmd=['echo', '1. GroundedSAM2物品挑選'],
                output='screen'
            ),
            ExecuteProcess(
                cmd=['echo', '2. 抓取並移動'],
                output='screen'
            ),
            ExecuteProcess(
                cmd=['echo', '3. 特定位置LLM確認'],
                output='screen'
            ),
            ExecuteProcess(
                cmd=['echo', '4. 完成分發'],
                output='screen'
            ),
            ExecuteProcess(
                cmd=['echo', 'Web界面: http://localhost:8000'],
                output='screen',
                condition=IfCondition(LaunchConfiguration('enable_gui'))
            ),
            ExecuteProcess(
                cmd=['echo', '系統狀態: ros2 topic echo /pharmacy/status'],
                output='screen'
            ),
            ExecuteProcess(
                cmd=['echo', '=============================='],
                output='screen'
            ),
        ]
    )
    
    return LaunchDescription([
        # 參數
        use_simulation_arg,
        enable_gui_arg,
        
        # 基礎系統
        camera_tf,
        camera_node,
        
        # 核心服務
        grounded_sam2_service,
        enhanced_grasp_node,
        llm_service,
        
        # 控制系統
        main_control_node,
        simple_pharmacy_system,
        
        # 用戶界面
        web_interface,
        
        # 完成通知
        startup_complete,
    ])