from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    nav2_pkg = get_package_share_directory('nav2_bringup')
    my_pkg = get_package_share_directory('my_robot_description')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': os.path.join(my_pkg, 'config', 'nav2_params.yaml'),
            'autostart': 'true'
        }.items()
    )

    return LaunchDescription([
        nav2_launch
    ])

