import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 获取你的包路径
    pkg_my_robot = get_package_share_directory('my_robot_description')
    
    # 2. 包含你刚才写好的仿真环境 (sim_base.launch.py)
    # 这会启动 Gazebo, Spawn Robot, Bridge, Robot State Publisher
    sim_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_my_robot, 'launch', 'sim_base.launch.py')
        )
    )

    # 3. 配置 SLAM (Slam Toolbox) - 用于建图
    # Nav2 需要地图。如果你是第一次跑，通常先跑 SLAM。
    # 这里直接使用 slam_toolbox 提供的在线异步建图启动文件
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': os.path.join(pkg_my_robot, 'config', 'mapper_params_online_async.yaml') 
            # 注意：如果没有这个 yaml，可以先删除这一行，使用默认参数
        }.items()
    )

    # 4. 配置 Nav2 (Navigation Stack)
    # 这会启动路径规划(Planner)、控制(Controller)、代价地图(Costmap)等
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': os.path.join(pkg_my_robot, 'config', 'nav2_params.yaml') 
            # 注意：如果还没有 params 文件，建议先去 nav2_bringup 复制一份默认的
        }.items()
    )

    # 5. Rviz2 (可视化)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_my_robot, 'rviz', 'nav2_view.rviz')], # 如果没有rviz文件可先去掉
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        sim_base_launch,
        slam_launch,
        nav2_launch,
        rviz_node
    ])
