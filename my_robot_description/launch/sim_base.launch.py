import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():

    pkg_name = 'my_robot_description'
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # ★★★ 修改 1: 获取存放 world 文件的包路径 ★★★
    # 注意：这里我们要找的是 my_robot_gazebo 这个包
    pkg_world = get_package_share_directory('my_robot_gazebo')
    world_file = os.path.join(pkg_world, 'worlds', 'simple_room.world')

    # 1. URDF / xacro
    xacro_file = os.path.join(
        get_package_share_directory(pkg_name),
        'urdf',
        'robot.urdf.xacro'
    )

    robot_description = Command(['xacro ', xacro_file])

    params = {
        'robot_description': robot_description,
        'use_sim_time': True
    }

    # 2. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[params],
        output='screen'
    )

    # 3. 启动 Ignition Gazebo 6
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            # ★★★ 修改 2: 加载我们的 simple_room.world ★★★
            'gz_args': f'-r -v 4 "{world_file}"'
        }.items(),
    )

    # 4. Spawn 机器人
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'my_robot',
            '-topic', 'robot_description',
            '-z', '0.2' # 把机器人稍微抬高一点，防止卡在地里
        ],
        output='screen'
    )

    # 5. ROS <-> Gazebo Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # 驱动命令 (ROS -> GZ)
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # 里程计 (GZ -> ROS)
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # TF 变换 (GZ -> ROS)
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/tf_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # 雷达 (GZ -> ROS)
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # 时钟 (GZ -> ROS)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # ★★★ 找回这一行！关节状态 (GZ -> ROS) ★★★
            # 没有它，轮子和机械臂的 TF 就会断开
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        output='screen'
    )
    
    # ★★★ 6. 静态 TF 补丁 (解决 Gazebo 传感器坐标系长名字问题) ★★★
    # 之前加在 nav2_sim 里了，但在 sim_base 里加更稳妥
    lidar_tf_fix = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_tf_fix',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'my_robot/base_footprint/lidar'],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn,
        bridge,
        lidar_tf_fix, # 别忘了返回这个节点
    ])
