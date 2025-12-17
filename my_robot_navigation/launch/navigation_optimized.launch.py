#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.actions import SetEnvironmentVariable, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, SetParameter, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
import os

def generate_launch_description():
    # 获取包路径
    pkg_nav2 = FindPackageShare('nav2_bringup')
    pkg_share = FindPackageShare('my_robot_navigation')
    
    # 参数配置
    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart', default='true')
    use_composition = LaunchConfiguration('use_composition', default='false')
    container_name = LaunchConfiguration('container_name', default='nav2_container')
    use_respawn = LaunchConfiguration('use_respawn', default='false')
    log_level = LaunchConfiguration('log_level', default='info')
    map_yaml_file = LaunchConfiguration('map')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    
    # 声明启动参数
    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use')
    
    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')
    
    declare_use_composition = DeclareLaunchArgument(
        'use_composition',
        default_value='false',
        description='Use composed bringup if True')
    
    declare_container_name = DeclareLaunchArgument(
        'container_name',
        default_value='nav2_container',
        description='the name of conatiner that nodes will load in if use composition')
    
    declare_use_respawn = DeclareLaunchArgument(
        'use_respawn',
        default_value='false',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')
    
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='log level')
    
    declare_map_yaml_file = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_share, 'maps', 'room_map.yaml'),
        description='Full path to map yaml file to load')
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2')
    
    # 设置环境变量
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')
    
    # 启动节点组
    bringup_cmd_group = GroupAction([
        # 地图服务器
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[{'use_sim_time': use_sim_time},
                       {'yaml_filename': map_yaml_file}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=[('/tf', 'tf'),
                       ('/tf_static', 'tf_static')]),
        
        # AMCL定位
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[params_file],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=[('/tf', 'tf'),
                       ('/tf_static', 'tf_static')]),
        
        # 生命周期管理器
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{'use_sim_time': use_sim_time,
                        'autostart': autostart,
                        'node_names': ['map_server', 'amcl']}]),
    ])
    
    # 导航节点组
    navigation_nodes = []
    if use_composition.perform(context) == 'false':
        # 独立节点模式
        navigation_nodes.extend([
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[params_file],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=[('/tf', 'tf'),
                          ('/tf_static', 'tf_static')]),
            
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[params_file],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=[('/tf', 'tf'),
                          ('/tf_static', 'tf_static')]),
            
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[params_file],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=[('/tf', 'tf'),
                          ('/tf_static', 'tf_static')]),
            
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[params_file],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=[('/tf', 'tf'),
                          ('/tf_static', 'tf_static')]),
            
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[params_file],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=[('/tf', 'tf'),
                          ('/tf_static', 'tf_static')]),
        ])
    
    # 代价地图服务器
    costmap_nodes = [
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='global_costmap',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[params_file],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=[('/tf', 'tf'),
                      ('/tf_static', 'tf_static')]),
        
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='local_costmap',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[params_file],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=[('/tf', 'tf'),
                      ('/tf_static', 'tf_static')]),
    ]
    
    # 导航生命周期管理器
    lifecycle_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'node_names': ['controller_server',
                                  'planner_server',
                                  'behavior_server',
                                  'bt_navigator',
                                  'waypoint_follower',
                                  'global_costmap',
                                  'local_costmap']}])
    
    # RViz2
    rviz_config_file = PathJoinSubstitution([pkg_share, 'config', 'rviz', 'navigation.rviz'])
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')
    
    return LaunchDescription([
        # 参数声明
        declare_namespace,
        declare_use_sim_time,
        declare_params_file,
        declare_autostart,
        declare_use_composition,
        declare_container_name,
        declare_use_respawn,
        declare_log_level,
        declare_map_yaml_file,
        declare_use_rviz,
        
        # 环境变量
        stdout_linebuf_envvar,
        
        # 延迟启动导航（等待定位就绪）
        TimerAction(
            period=2.0,
            actions=[
                bringup_cmd_group,
            ]
        ),
        
        TimerAction(
            period=5.0,
            actions=navigation_nodes + costmap_nodes + [lifecycle_navigation]
        ),
        
        # RViz2
        rviz_node,
    ])
