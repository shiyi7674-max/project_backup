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
            'gz_args': '-r -v 4 empty.sdf'
        }.items(),
    )

    # 4. Spawn 机器人
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'my_robot',
            '-topic', 'robot_description',
            '-z', '0.2'
        ],
        output='screen'
    )

    # 5. 【关键修正】ROS <-> Gazebo Bridge（Ignition 6 正确写法）
    # 5. ROS <-> Gazebo Bridge
    # 5. ROS <-> Gazebo Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # 1. 速度控制 (ROS -> Gazebo)
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            
            # 2. 里程计 (Gazebo -> ROS)
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            
            # 3. TF 坐标变换 (Gazebo -> ROS) - 必须要有！
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/tf_static@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            
            # 4. 激光雷达 (Gazebo -> ROS)
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            
            # 5. IMU (Gazebo -> ROS)
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            
            # 6. ★★★ 新增：关节状态 (Gazebo -> ROS) ★★★
            # 缺少这个会导致机械臂和轮子报错 "No transform"
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            
            # 7. 仿真时钟 (Gazebo -> ROS)
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
        ],
        output='screen'
    )

    return LaunchDescription([
    	gazebo,
        robot_state_publisher,
        spawn,
        bridge,
    ])

