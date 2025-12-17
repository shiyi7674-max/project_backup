import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_nav = get_package_share_directory('my_robot_navigation')
    pkg_desc = get_package_share_directory('my_robot_description')
    nav2_params = os.path.join(pkg_nav, 'config/nav2_params/nav2_params.yaml')
    rviz_config = os.path.join(pkg_nav, 'config/rviz/navigation.rviz')

    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen',
             parameters=[os.path.join(pkg_desc, 'urdf/robot.urdf.xacro'),
                         {'use_sim_time': True}]),
        Node(package='nav2_bringup', executable='nav2_bringup_launch',
             name='nav2', output='screen', parameters=[nav2_params]),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', rviz_config])
    ])
