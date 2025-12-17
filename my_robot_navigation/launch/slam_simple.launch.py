#!/usr/bin/env python3
# 简化版SLAM启动文件
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 包路径
    pkg_navigation = get_package_share_directory('my_robot_navigation')
    pkg_description = get_package_share_directory('my_robot_description')
    
    # 参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    slam_params_file = PathJoinSubstitution([
        pkg_navigation, 'config', 'slam_toolbox_params.yaml'
    ])
    
    # URDF文件路径
    urdf_file = os.path.join(pkg_description, 'urdf', 'robot.urdf.xacro')
    
    return LaunchDescription([
        # 参数声明
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=slam_params_file,
            description='Full path to the SLAM parameters file'),
        
        # 1. 启动机器人状态发布器
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': Command([
                    'xacro ', urdf_file
                ])
            }]),
        
        # 2. 启动SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_params_file,
                {'use_sim_time': use_sim_time}
            ],
            remappings=[
                ('/scan', '/scan'),
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')
            ]),
        
        # 3. 启动RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-d', PathJoinSubstitution([
                pkg_navigation, 'config', 'slam.rviz'
            ])]),
    ])
