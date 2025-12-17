import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_nav = get_package_share_directory('my_robot_navigation')
    rviz_config = os.path.join(pkg_nav, 'config/rviz/navigation.rviz')
    return LaunchDescription([
        Node(package='rviz2', executable='rviz2', name='rviz2',
             output='screen', arguments=['-d', rviz_config])
    ])
