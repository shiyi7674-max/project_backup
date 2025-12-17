import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 定义包路径
    pkg_description = get_package_share_directory('my_robot_description')
    pkg_navigation = get_package_share_directory('my_robot_navigation')
    
    # 2. 启动仿真环境 (Gazebo + Robot + Bridge)
    # 调用 description 包里的 sim_base
    sim_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, 'launch', 'sim_base.launch.py')
        )
    )

    # 3. 静态 TF 修复 (解决 Gazebo 传感器坐标系长名字问题)
    # 强行把 my_robot/base_footprint/lidar 连接到 lidar_link
    lidar_tf_fix = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_tf_fix',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'my_robot/base_footprint/lidar'],
        output='screen'
    )

    # 4. 启动 SLAM (使用 slam_toolbox)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            # 如果你有 mapper_params，可以在这里指定，没有就用默认
            # 'slam_params_file': os.path.join(pkg_navigation, 'config', 'mapper_params_online_async.yaml') 
        }.items()
    )

    # 5. 启动 Nav2 (使用我们刚才写好的 nav2_params.yaml)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': os.path.join(pkg_navigation, 'params', 'nav2_params.yaml') 
        }.items()
    )

    # 6. Rviz2 (使用 navigation 包里的配置)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_navigation, 'rviz', 'nav2_view.rviz')],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        sim_base_launch,
        lidar_tf_fix,    # 别忘了这个关键补丁
        slam_launch,
        nav2_launch,
        rviz_node
    ])
