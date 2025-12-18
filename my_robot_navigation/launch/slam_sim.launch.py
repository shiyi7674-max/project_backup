import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_desc = get_package_share_directory('my_robot_description')
    pkg_slam = get_package_share_directory('slam_toolbox')
    pkg_nav  = get_package_share_directory('my_robot_navigation')

    return LaunchDescription([

        # 1. 仿真 / 机器人 / TF / 传感器
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_desc, 'launch', 'sim_base.launch.py')
            )
        ),

        # 2. SLAM Toolbox（只做建图）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_slam, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': os.path.join(
                    pkg_nav,
                    'config',
                    'maps',
                    'mapper_params_online_async.yaml'
                )
            }.items()
        ),

        # 3. RViz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav, 'launch', 'rviz_launch.py')
            )
        )
    ])

