import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_nav = get_package_share_directory('my_robot_navigation')
    pkg_desc = get_package_share_directory('my_robot_description')
    robot_yaml = os.path.join(pkg_nav, 'config/robots/my_robot.yaml')

    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen',
             parameters=[os.path.join(pkg_desc, 'urdf/robot.urdf.xacro'),
                         {'use_sim_time': True}]),
        Node(package='slam_toolbox', executable='sync_slam_toolbox_node',
             name='slam_toolbox', output='screen', parameters=[robot_yaml, {'use_sim_time': True}]),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', os.path.join(pkg_nav, 'config/rviz/slam.rviz')])
    ])
